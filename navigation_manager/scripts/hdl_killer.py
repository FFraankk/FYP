#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import os
import yaml
import rospkg
import numpy as np
from hdl_localization.msg import ScanMatchingStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

class HdlGovernorJetson:
    def __init__(self):
        rospy.init_node('hdl_governor_node', anonymous=False)

        # --- 1. 参数配置 ---
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'camera_init')
        self.odom_frame = rospy.get_param('~odom_frame', 'aft_mapped') 
        
        self.inlier_thresh = rospy.get_param('~inlier_fraction_thresh', 0.95)
        self.error_thresh = rospy.get_param('~matching_error_thresh', 0.04)
        
        self.tf_rate = rospy.get_param('~tf_pub_rate', 30.0)
        self.logic_rate = 3.0   
        self.recalib_duration = rospy.Duration(10.0) 

        # --- 2. 状态变量 ---
        self.current_idx = 0
        self.wait_count = 0     
        self.best_inlier_seen = 0.0 
        self.last_fix_time = rospy.Time(0)
        self.last_recalib_trigger = rospy.Time.now()
        
        self.best_trans = None
        self.best_rot = None
        self.is_localized = False 

        # --- 3. 初始化 ---
        self.load_waypoints()
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.init_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        try:
            rospy.loginfo("[Governor] 等待 /relocalize 服务...")
            rospy.wait_for_service('/relocalize', timeout=2.0)
            self.reloc_srv = rospy.ServiceProxy('/relocalize', Empty)
        except:
            self.reloc_srv = None
            rospy.logwarn("[Governor] 未发现重定位服务")

        self.status_sub = rospy.Subscriber('/status', ScanMatchingStatus, self.status_callback, queue_size=10)

        # 定时器
        rospy.Timer(rospy.Duration(1.0 / self.tf_rate), self.publish_tf_loop)
        rospy.Timer(rospy.Duration(self.logic_rate), self.main_logic_loop)

        rospy.loginfo("[Governor] 系统就绪：候选点模式 -> 锁定追踪模式")

    def load_waypoints(self):
        try:
            rp = rospkg.RosPack()
            pkg_path = rp.get_path('navigation_manager')
            yaml_path = os.path.join(pkg_path, 'config', 'waypoints.yaml')
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
                self.candidates = config.get('start_candidates', [])
            rospy.loginfo(f"[Governor] 加载了 {len(self.candidates)} 个点")
        except:
            self.candidates = []
            rospy.logerr("[Governor] 无法加载候选点文件")

    def main_logic_loop(self, event):
        now = rospy.Time.now()

        if not self.is_localized:
            if not self.candidates:
                self.call_relocalize()
                return

            if self.wait_count < 3:
                p = self.candidates[self.current_idx]
                if self.wait_count == 0:
                    rospy.loginfo(f"--- 尝试候选点 {self.current_idx} ---")
                    self.publish_initial_pose(p)
                self.wait_count += 1
            else:
                if self.current_idx == len(self.candidates) - 1:
                    rospy.logwarn("[保底] 全局重定位触发...")
                    self.call_relocalize()
                    self.wait_count = -2 
                else:
                    self.wait_count = 0
                self.current_idx = (self.current_idx + 1) % len(self.candidates)
                self.best_inlier_seen = 0.0
        else:
            # 追踪模式：定期校准
            if (now - self.last_recalib_trigger) > self.recalib_duration:
                self.trigger_calibration_pose()
                self.last_recalib_trigger = now

    def trigger_calibration_pose(self):
        try:
            # 获取最新 map -> aft_mapped 投喂给 HDL
            if self.listener.canTransform(self.map_frame, self.odom_frame, rospy.Time(0)):
                (trans, rot) = self.listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                self.publish_initial_pose([trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]])
                rospy.loginfo("[校准] 已同步当前位姿")
        except: pass

    def call_relocalize(self):
        if self.reloc_srv:
            try: self.reloc_srv()
            except: pass

    def publish_initial_pose(self, p):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p[0:3]
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = p[3:7]
        cov = [0.0] * 36
        cov[0] = 0.1; cov[7] = 0.1; cov[35] = 0.05
        msg.pose.covariance = cov
        self.init_pub.publish(msg)

    def status_callback(self, msg):
        now = rospy.Time.now()
        if msg.inlier_fraction > self.best_inlier_seen:
            self.best_inlier_seen = msg.inlier_fraction

        if msg.inlier_fraction > self.inlier_thresh and msg.matching_error < self.error_thresh:
            try:
                # 关键：这里用 waitForTransform 增加一点鲁棒性
                self.listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.1))
                (trans, rot) = self.listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
                
                self.best_trans = trans
                self.best_rot = rot
                self.last_fix_time = now
                
                if not self.is_localized:
                    rospy.loginfo(">>>>> 定位锁定成功 <<<<<")
                    self.is_localized = True
            except: pass

    def publish_tf_loop(self, event):
        if self.best_trans and self.best_rot:
            self.br.sendTransform(
                self.best_trans, self.best_rot, 
                rospy.Time.now(), 
                self.base_frame, self.map_frame
            )

if __name__ == '__main__':
    try:
        HdlGovernorJetson()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass