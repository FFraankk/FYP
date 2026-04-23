#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import rospy
import rospkg
import tf
import subprocess  
from hdl_localization.msg import ScanMatchingStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32  # 【新增】导入 Float32 消息类型
from std_srvs.srv import Empty, SetBool, SetBoolResponse

class HdlGovernorJetson:
    def __init__(self):
        rospy.init_node('hdl_governor_node', anonymous=False)

        # --- 1. 参数配置 ---
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.hdl_odom_frame = rospy.get_param('~hdl_odom_frame', 'camera_init') 
        self.hdl_node_name = rospy.get_param('~hdl_node_name', '/hdl_localization_nodelet')

        self.inlier_thresh = rospy.get_param('~inlier_fraction_thresh', 0.95)
        self.error_thresh = rospy.get_param('~matching_error_thresh', 0.04)
        self.logic_rate = 3.0  

        # --- 2. 状态变量 ---
        self.current_idx = 0
        self.wait_count = 0     
        self.is_localized = False 
        self.is_paused = False 
        self.lost_counter = 0
        self.max_lost_frames = 120
        
        self.tf_frozen = False        
        self.frozen_trans = None      # 保存冻结时的平移
        self.frozen_rot = None        # 保存冻结时的旋转
        self.healthy_counter = 0      # 健康帧连续计数器

        # 【新增】高度补偿变量与订阅，接收 floor_manager 发来的欺骗指令
        self.z_offset = 0.0
        rospy.Subscriber('/set_z_offset', Float32, self.z_offset_cb)

        # --- 3. 核心初始化 ---
        self.candidates = self._load_waypoints()
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster() # 自己当广播员

        self.init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.pause_srv = rospy.Service('/set_hdl_pause', SetBool, self.handle_pause)
        self.reloc_srv = self._init_reloc_service()
        self.status_sub = rospy.Subscriber('/status', ScanMatchingStatus, self.status_callback, queue_size=10)

        # 业务逻辑定时器
        rospy.Timer(rospy.Duration(1.0 / self.logic_rate), self.main_logic_loop)
        
        # 高频 TF 发布定时器 (50Hz)，保证无缝接力
        rospy.Timer(rospy.Duration(0.02), self.publish_frozen_tf)

        rospy.loginfo("[Governor] 初始化完成：平滑夺权与高度欺骗机制已就绪")

    def z_offset_cb(self, msg):
        """【新增】接收 Z 轴偏移指令"""
        self.z_offset = msg.data
        rospy.loginfo(f"[Governor] 接收到高度欺骗指令，当前 TF Z轴偏移量设置为: {self.z_offset}m")

    def _load_waypoints(self):
        try:
            rp = rospkg.RosPack()
            yaml_path = os.path.join(rp.get_path('navigation_manager'), 'config', 'waypoints.yaml')
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
                nodes = config.get('start_candidates', [])
                rospy.loginfo(f"[Governor] 成功加载 {len(nodes)} 个候选点")
                return nodes
        except Exception as e:
            rospy.logerr(f"[Governor] 候选点加载失败: {e}")
            return []

    def _init_reloc_service(self):
        try:
            rospy.wait_for_service('/relocalize', timeout=2.0)
            return rospy.ServiceProxy('/relocalize', Empty)
        except rospy.ROSException:
            return None

    def handle_pause(self, req):
        self.is_paused = req.data
        msg = "暂停" if self.is_paused else "恢复"
        if not self.is_paused:
            self.wait_count = 0
            self.lost_counter = 0
        rospy.loginfo(f"[Governor] 指令切换: {msg}")
        return SetBoolResponse(success=True, message=f"State: {msg}")

    def main_logic_loop(self, event):
        if self.is_paused or self.is_localized or self.tf_frozen:
            return

        if not self.candidates:
            self.call_relocalize()
            return

        if self.wait_count < 3:
            if self.wait_count == 0:
                rospy.loginfo(f"[Governor] 尝试候选点 [{self.current_idx}]")
                self.publish_initial_pose(self.candidates[self.current_idx])
            self.wait_count += 1
        else:
            if self.current_idx >= len(self.candidates) - 1:
                self.call_relocalize()
                self.wait_count = -2 
            else:
                self.wait_count = 0
            self.current_idx = (self.current_idx + 1) % len(self.candidates)

    def publish_initial_pose(self, p):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p[0:3]
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = p[3:7]
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = 0.1, 0.1, 0.05
        msg.pose.covariance = cov
        self.init_pub.publish(msg)

    def call_relocalize(self):
        if self.reloc_srv:
            try: self.reloc_srv()
            except rospy.ServiceException: pass

    def freeze_localization(self):
        """执行夺权：抓取 -> 启动自身广播 -> 后台杀掉原节点"""
        if self.tf_frozen: 
            return
            
        try:
            rospy.loginfo(f"[Governor] 正在抓取锁定瞬间的坐标...")
            self.tf_listener.waitForTransform(self.map_frame, self.hdl_odom_frame, rospy.Time(0), rospy.Duration(3.0))
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.hdl_odom_frame, rospy.Time(0))

            self.frozen_trans = trans
            self.frozen_rot = rot
            self.tf_frozen = True 
            
            rospy.loginfo(f"[Governor] 抓取成功！后台强制关闭原节点: {self.hdl_node_name}")
            subprocess.Popen(["rosnode", "kill", self.hdl_node_name])
            
            rospy.loginfo(f">>>>> 无缝夺权完成！位置已冻结！ <<<<<")

            try:
                clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
                clear_srv()
            except Exception:
                pass

        except Exception as e:
            rospy.logerr(f"[Governor] 冻结定位失败: {e}")
            self.is_localized = False 
            self.tf_frozen = False

    def publish_frozen_tf(self, event):
        """高频持续发布死坐标，扮演假节点"""
        if self.tf_frozen and self.frozen_trans and self.frozen_rot:
            # 【核心修改】应用高度欺骗偏移量
            trans = list(self.frozen_trans)
            trans[2] += self.z_offset 
            
            self.tf_broadcaster.sendTransform(
                trans,
                self.frozen_rot,
                rospy.Time.now() + rospy.Duration(0.05), 
                self.hdl_odom_frame,
                self.map_frame
            )

    def status_callback(self, msg):
        if self.is_paused or self.tf_frozen:
            return 

        is_healthy = (msg.inlier_fraction > self.inlier_thresh and 
                      msg.matching_error < self.error_thresh)

        if is_healthy:
            self.lost_counter = 0
            self.healthy_counter += 1 
            
            if self.healthy_counter > 15: 
                if not self.is_localized:
                    rospy.loginfo(">>>>> 定位已充分收敛锁定：准备执行无缝冻结 <<<<<")
                    self.is_localized = True
                    self.freeze_localization() 
            elif self.healthy_counter % 5 == 0:
                rospy.loginfo(f"[Governor] 定位收敛中... ({self.healthy_counter}/15)")
        else:
            self.healthy_counter = 0 
            
            if self.is_localized:
                self.lost_counter += 1
                if self.lost_counter > self.max_lost_frames:
                    self.is_localized = False 
                    self.lost_counter = 0

if __name__ == '__main__':
    try:
        HdlGovernorJetson()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass