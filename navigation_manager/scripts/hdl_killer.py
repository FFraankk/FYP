#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import os
import yaml
import rospkg
from hdl_localization.msg import ScanMatchingStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

class HdlKillerJetson:
    def __init__(self):
        rospy.init_node('hdl_killer_node', anonymous=False)

        # --- 1. 参数化配置 - 遵循用户习惯: base_frame 为里程计系(camera_init)，odom_frame 为机器人系(aft_mapped) ---
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'camera_init') # Odom Origin (Parent)
        self.odom_frame = rospy.get_param('~odom_frame', 'aft_mapped')   # Robot Body (Child)
        
        # 收敛阈值
        self.inlier_thresh = rospy.get_param('~inlier_fraction_thresh', 0.90)
        self.error_thresh = rospy.get_param('~matching_error_thresh', 0.05)
        
        # 节点清理列表
        self.nodes_to_kill = rospy.get_param('~nodes_to_kill', [
            '/velodyne_nodelet_manager', 
            '/hdl_global_localization'
        ])
        
        # 频率控制
        self.tf_rate = rospy.get_param('~tf_pub_rate', 50.0)
        self.logic_rate = rospy.get_param('~logic_rate', 5.0)

        # --- 2. 内部状态变量 ---
        self.alignment_locked = False
        self.locked_transform = None
        self.current_idx = 0
        self.loop_counter = 0
        
        # --- 3. 路径与配置加载 ---
        try:
            rp = rospkg.RosPack()
            pkg_path = rp.get_path('navigation_manager')
            yaml_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')
            
            if not os.path.exists(yaml_path):
                rospy.logerr("[HDL_KILLER] 致命错误: 配置文件不存在于 %s", yaml_path)
                self.candidates = []
            else:
                with open(yaml_path, 'r') as f:
                    config = yaml.safe_load(f)
                    self.candidates = config.get('start_candidates', [])
                    if not self.candidates:
                        rospy.logwarn("[HDL_KILLER] YAML中没有找到 start_candidates 列表。")
        except Exception as e:
            rospy.logerr("[HDL_KILLER] 配置文件加载异常: %s", str(e))
            self.candidates = []

        # --- 4. ROS 通信接口 ---
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        
        self.init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        rospy.loginfo("[HDL_KILLER] 正在等待 /relocalize 服务 (timeout=10s)...")
        try:
            rospy.wait_for_service('/relocalize', timeout=10.0)
            self.reloc_srv = rospy.ServiceProxy('/relocalize', Empty)
            rospy.loginfo("[HDL_KILLER] 服务连接成功。")
        except rospy.ROSException:
            self.reloc_srv = None
            rospy.logerr("[HDL_KILLER] 无法连接到 /relocalize 服务！")

        self.status_sub = rospy.Subscriber('/status', ScanMatchingStatus, self.status_callback, queue_size=10)

        # --- 5. 任务调度定时器 ---
        rospy.Timer(rospy.Duration(1.0 / self.tf_rate), self.publish_tf_loop)
        rospy.Timer(rospy.Duration(self.logic_rate), self.search_logic_loop)

        rospy.loginfo("[HDL_KILLER] 混合重定位系统已就绪。正在等待收敛信号...")

    def search_logic_loop(self, event):
        if self.alignment_locked:
            return

        self.loop_counter += 1
        
        # --- 策略 A: 坐标投喂 ---
        if self.candidates:
            p = self.candidates[self.current_idx]
            self.publish_initial_pose(p)
            rospy.loginfo("[HDL_KILLER] [策略A] 投喂候选点 %d/%d", self.current_idx + 1, len(self.candidates))
            self.current_idx = (self.current_idx + 1) % len(self.candidates)

        # --- 策略 B: 全局重定位服务 (每 3 次循环触发一次，默认 15s) ---
        if self.loop_counter % 3 == 0:
            if self.reloc_srv:
                try:
                    self.reloc_srv()
                    rospy.logwarn("[HDL_KILLER] [策略B] 触发全局重定位服务")
                except Exception as e:
                    rospy.logerr("[HDL_KILLER] 调用全局服务失败: %s", str(e))

    def publish_initial_pose(self, p):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        
        try:
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p[0:3]
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = p[3:7]
            
            cov = [0.0] * 36
            cov[0] = 0.25; cov[7] = 0.25; cov[14] = 0.01; cov[35] = 0.1
            msg.pose.covariance = cov
            
            self.init_pub.publish(msg)
        except IndexError:
            rospy.logerr("[HDL_KILLER] 候选点格式错误！")

    def status_callback(self, msg):
        if self.alignment_locked:
            return

        # 判定收敛标准
        if msg.inlier_fraction > self.inlier_thresh and msg.matching_error < self.error_thresh:
            rospy.logwarn("[HDL_KILLER] 判定收敛！Inlier: %.3f, Error: %.4f", 
                          msg.inlier_fraction, msg.matching_error)
            self.execute_lock_and_kill()

    def execute_lock_and_kill(self):
        try:
            # 不要直接 sleep，给 TF 监听器一点缓冲时间
            target_time = rospy.Time(0) 
            found = False
            
            # 尝试在 5 秒内持续查找，而不是只查一次
            for i in range(10): 
                if self.listener.canTransform(self.map_frame, self.base_frame, target_time):
                    (trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, target_time)
                    self.locked_transform = (trans, rot)
                    self.alignment_locked = True
                    found = True
                    break
                rospy.sleep(0.1) # 短暂重试
                
            if found:
                rospy.logerr("★★★ [HDL_KILLER] 成功锁定变换！ ★★★")
                nodes_str = " ".join(self.nodes_to_kill)
                os.system(f"rosnode kill {nodes_str}")
            else:
                rospy.logerr("[HDL_KILLER] 即使重试也无法获取 TF (map -> %s)", self.base_frame)

        except Exception as e:
            rospy.logerr("[HDL_KILLER] 锁定异常: %s", str(e))
            
    def publish_tf_loop(self, event):
        if self.alignment_locked and self.locked_transform:
            try:
                # 持续发布 map -> base_frame (camera_init)
                self.br.sendTransform(
                    self.locked_transform[0], 
                    self.locked_transform[1], 
                    rospy.Time.now(), 
                    self.base_frame, 
                    self.map_frame
                )
            except:
                pass

if __name__ == '__main__':
    try:
        HdlKillerJetson()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass