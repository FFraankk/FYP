#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import rospy
import rospkg
from hdl_localization.msg import ScanMatchingStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty, SetBool, SetBoolResponse

class HdlGovernorJetson:
    def __init__(self):
        rospy.init_node('hdl_governor_node', anonymous=False)

        # --- 1. 参数配置 ---
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.inlier_thresh = rospy.get_param('~inlier_fraction_thresh', 0.95)
        self.error_thresh = rospy.get_param('~matching_error_thresh', 0.04)
        self.logic_rate = 3.0  

        # --- 2. 状态变量 ---
        self.current_idx = 0
        self.wait_count = 0     
        self.best_inlier_seen = 0.0 
        self.is_localized = False 
        self.is_paused = False 
        self.lost_counter = 0
        self.max_lost_frames = 120

        # --- 3. 核心初始化 ---
        self.candidates = self._load_waypoints()
        
        # 发布与服务
        self.init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.pause_srv = rospy.Service('/set_hdl_pause', SetBool, self.handle_pause)
        
        # 动态绑定重定位服务
        self.reloc_srv = self._init_reloc_service()

        # 订阅状态
        self.status_sub = rospy.Subscriber('/status', ScanMatchingStatus, self.status_callback, queue_size=10)

        # 定时器触发逻辑
        rospy.Timer(rospy.Duration(1.0 / self.logic_rate), self.main_logic_loop)
        rospy.loginfo("[Governor] 初始化完成：容错机制已就绪")

    def _load_waypoints(self):
        """内部方法：加载路径点"""
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
        """安全初始化重定位服务"""
        try:
            rospy.loginfo("[Governor] 正在寻找 /relocalize 服务...")
            rospy.wait_for_service('/relocalize', timeout=2.0)
            return rospy.ServiceProxy('/relocalize', Empty)
        except rospy.ROSException:
            rospy.logwarn("[Governor] 未发现重定位服务，全局搜索功能受限")
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
        if self.is_paused or self.is_localized:
            return

        if not self.candidates:
            self.call_relocalize()
            return

        # 轮询逻辑
        if self.wait_count < 3:
            if self.wait_count == 0:
                rospy.loginfo(f"[Governor] 尝试候选点 [{self.current_idx}]")
                self.publish_initial_pose(self.candidates[self.current_idx])
            self.wait_count += 1
        else:
            # 检查是否完成一轮
            if self.current_idx >= len(self.candidates) - 1:
                rospy.logwarn("[Governor] 所有点尝试完毕，执行全局重定位")
                self.call_relocalize()
                self.wait_count = -2 # 给全局搜索留点宽限期
            else:
                self.wait_count = 0
            
            self.current_idx = (self.current_idx + 1) % len(self.candidates)

    def publish_initial_pose(self, p):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p[0:3]
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = p[3:7]
        
        # 设定协方差：x, y, yaw
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = 0.1, 0.1, 0.05
        msg.pose.covariance = cov
        self.init_pub.publish(msg)

    def call_relocalize(self):
        if self.reloc_srv:
            try:
                self.reloc_srv()
            except rospy.ServiceException:
                pass

    def status_callback(self, msg):
        if self.is_paused:
            return

        is_healthy = (msg.inlier_fraction > self.inlier_thresh and 
                      msg.matching_error < self.error_thresh)

        if is_healthy:
            self.lost_counter = 0
            if not self.is_localized:
                rospy.loginfo(">>>>> 定位锁定：切换至巡航模式 <<<<<")
                self.is_localized = True
        else:
            if self.is_localized:
                self.lost_counter += 1
                if self.lost_counter > self.max_lost_frames:
                    rospy.logerr("!!!!! 严重失配：判定为绑架，重进唤醒模式 !!!!!")
                    self.is_localized = False
                    self.lost_counter = 0

if __name__ == '__main__':
    try:
        HdlGovernorJetson()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass