#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import os
import yaml
import rospkg
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalStatus

class FloorManager:
    def __init__(self):
        rospy.init_node('floor_manager_node')
        
        # 参数化配置 - 遵循用户习惯: base_frame 为里程计系，odom_frame 为机器人系
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'camera_init') # Odom Origin (Parent)
        self.odom_frame = rospy.get_param('~odom_frame', 'aft_mapped')   # Robot Body (Child)
        self.move_base_ns = rospy.get_param('~move_base_ns', 'move_base')

        self.rp = rospkg.RosPack()
        self.package_path = self.rp.get_path('navigation_manager')
        self.load_config()

        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient(self.move_base_ns, MoveBaseAction)
        rospy.loginfo("Floor Manager: 正在等待 move_base 服务器...")
        if not self.move_base.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Floor Manager: 无法连接到 move_base 服务器！")
        
        try:
            self.clear_srv = rospy.ServiceProxy(f'/{self.move_base_ns}/clear_costmaps', Empty)
        except Exception as e:
            rospy.logwarn(f"Floor Manager: 无法连接清空代价地图服务: {e}")
            self.clear_srv = None

        # 内部状态
        self.curr_pos = None # [x, y, z]
        self.curr_floor = None

        # 启动位姿监听定时器 (10Hz)
        rospy.Timer(rospy.Duration(0.1), self.update_pose_from_tf)
        rospy.loginfo("Floor Manager: 业务节点就绪（TF监听模式）。")

    def load_config(self):
        config_path = os.path.join(self.package_path, 'config', 'waypoints.yaml')
        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
                self.wp_4f = cfg['floor_4']
                self.wp_3f = cfg['floor_3']
                self.floor_th = cfg['settings']['floor_threshold']
                self.dist_th = cfg['settings']['dist_threshold']
        except Exception as e:
            rospy.logerr(f"Floor Manager: 加载配置文件失败: {e}")
            rospy.signal_shutdown("Config load failure")

    def update_pose_from_tf(self, event):
        try:
            # 监听 map 到机器人本体 odom_frame (aft_mapped) 的变换
            if self.tf_listener.canTransform(self.map_frame, self.odom_frame, rospy.Time(0)):
                (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                self.curr_pos = trans
                self.curr_floor = 4 if trans[2] > self.floor_th else 3
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(10, f"Floor Manager: TF 监听异常: {e}")

    def switch_map(self, floor):
        name = self.wp_4f['map_name'] if floor == 4 else self.wp_3f['map_name']
        path = os.path.join(self.package_path, 'maps', name)
        rospy.loginfo("【切图】正在尝试加载: %s", name)
        os.system("rosnode kill /map_server") 
        rospy.sleep(0.5)
        os.system(f"rosrun map_server map_server {path} __name:=map_server &")
        rospy.sleep(2.0)

    def get_dist(self, target):
        if self.curr_pos is None: return 1000.0
        return math.sqrt((self.curr_pos[0]-target[0])**2 + (self.curr_pos[1]-target[1])**2)

    def wait_arrival(self, floor):
        target = self.wp_3f['stairs_point'] if floor == 3 else self.wp_4f['stairs_point']
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.curr_floor == floor and self.get_dist(target) < self.dist_th:
                rospy.loginfo("【验证通过】已跨越楼层阈值并到达目标层。")
                break
            rate.sleep()

    def send_goal(self, p):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z = p[0:3]
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = p[3:7]
        
        rospy.loginfo(f"Floor Manager: 正在发送导航目标点...")
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()
        return self.move_base.get_state()

    def run_mission(self, target_floor, target_key):
        rospy.loginfo(f"Floor Manager: 开始执行任务 -> 楼层: {target_floor}, 目标: {target_key}")
        while self.curr_floor is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(5, "Floor Manager: 等待初始定位信息...")
            rospy.sleep(1.0)

        target_cfg = self.wp_4f if target_floor == 4 else self.wp_3f
        if self.curr_floor != target_floor:
            rospy.loginfo(f"Floor Manager: 当前在 {self.curr_floor} 层，目标在 {target_floor} 层，准备前往转换点...")
            cur_stairs = self.wp_4f['stairs_point'] if self.curr_floor == 4 else self.wp_3f['stairs_point']
            if self.send_goal(cur_stairs) == GoalStatus.SUCCEEDED:
                rospy.loginfo("已到楼梯口，等待换层...")
                self.wait_arrival(target_floor)
                self.switch_map(target_floor)
                if self.clear_srv:
                    try: self.clear_srv()
                    except: pass
                rospy.sleep(1.0)
                self.send_goal(target_cfg[target_key])
        else:
            rospy.loginfo(f"Floor Manager: 目标层即为当前层，直接前往目标点。")
            self.send_goal(target_cfg[target_key])

if __name__ == '__main__':
    try:
        mgr = FloorManager()
        rospy.sleep(2.0)
        target_f = rospy.get_param('~initial_target_floor', 3)
        target_k = rospy.get_param('~initial_target_key', 'office_goal')
        mgr.run_mission(target_f, target_k)
    except rospy.ROSInterruptException:
        pass