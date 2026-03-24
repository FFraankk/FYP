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
from std_srvs.srv import Empty, SetBool
from actionlib_msgs.msg import GoalStatus

class FloorManager:
    def __init__(self):
        rospy.init_node('floor_manager_node')
        
        # 参数化配置 - 遵循用户习惯: base_frame 为里程计系，odom_frame 为机器人系
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'camera_init') # Odom Origin (Parent)
        self.odom_frame = rospy.get_param('~odom_frame', 'aft_mapped')  # Robot Body (Child)
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
        
        # 【新增 1】爬楼梯状态锁，防止定时器在爬楼期间乱改数据
        self.is_climbing = False 

        # 启动位姿监听定时器 (10Hz)
        rospy.Timer(rospy.Duration(0.1), self.update_pose_from_tf)
        rospy.loginfo("Floor Manager: 业务节点就绪（TF监听模式）。")

        # ----------------- 服务连接区 -----------------
        rospy.loginfo("Floor Manager: 正在连接底层控制与辅助服务...")
        
        # 1. 连接爬楼梯服务
        try:
            rospy.wait_for_service('/execute_stair_climb', timeout=5.0)
            self.climb_srv = rospy.ServiceProxy('/execute_stair_climb', SetBool)
            rospy.loginfo("Floor Manager: 启动爬楼梯服务！")
        except rospy.ROSException:
            rospy.logwarn("Floor Manager: 未检测到爬楼梯服务，请确保 stair_climber 节点已启动！")

        # 【新增 2】2. 连接 HDL Governor 的暂停服务
        try:
            rospy.wait_for_service('/set_hdl_pause', timeout=3.0)
            self.pause_hdl_srv = rospy.ServiceProxy('/set_hdl_pause', SetBool)
            rospy.loginfo("Floor Manager: HDL 暂停服务连接成功。")
        except rospy.ROSException:
            self.pause_hdl_srv = None
            rospy.logwarn("Floor Manager: 未检测到 /set_hdl_pause 服务，请确保 hdl_governor_node 已启动！")
        # ----------------------------------------------

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
        # 【新增 3】爬楼期间，直接无视 TF 传来的绝对高度，彻底杜绝状态闪跳Bug
        if self.is_climbing:
            return 
            
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
        
        rospy.loginfo("【联动】检测到楼层变化，正在平滑切换 2D 导航图...")
        
        # 1. 先暂停当前导航目标，防止 move_base 在没图的时候报错
        self.move_base.cancel_all_goals()
        
        # 2. 清理一下 costmap，避免残留上一层的障碍物
        if self.clear_srv:
            try: self.clear_srv()
            except: pass
            
        # 3. 切换 2D 地图（不影响 HDL，因为 3D Map 没动）
        os.system("rosnode kill /map_server") 
        rospy.sleep(0.5)
        os.system(f"rosrun map_server map_server {path} __name:=map_server &")
        
        # 4. 给 move_base 一点时间重新加载静态层
        rospy.sleep(1.5) 
        
        # 5. 再次清理代价地图，确保新图生效
        if self.clear_srv:
            try: self.clear_srv()
            except: pass
            
        rospy.loginfo("【联动】2D 导航环境重载完成。")

    def send_goal(self, p):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z = p[0:3]
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = p[3:7]
        
        rospy.loginfo("Floor Manager: 正在发送导航目标点...")
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()
        return self.move_base.get_state()

    def run_mission(self, target_floor, target_key):
        rospy.loginfo(f"Floor Manager: 开始执行任务 -> 楼层: {target_floor}, 目标: {target_key}")
        
        # 0. 等待初始定位
        while self.curr_floor is None and not rospy.is_shutdown():
            rospy.loginfo_throttle(5, "Floor Manager: 等待初始定位信息...")
            rospy.sleep(1.0)

        target_cfg = self.wp_4f if target_floor == 4 else self.wp_3f
        
        if self.curr_floor != target_floor:
            rospy.loginfo(f"Floor Manager: 当前在 {self.curr_floor} 层，目标在 {target_floor} 层，准备前往转换点...")
            cur_stairs = self.wp_4f['stairs_point'] if self.curr_floor == 4 else self.wp_3f['stairs_point']
            
            # 1. 导航到当前楼层的楼梯口
            if self.send_goal(cur_stairs) == GoalStatus.SUCCEEDED:
                rospy.loginfo("已到楼梯口，等待换层...")
                
                # 必须先取消 move_base 的目标，剥夺它的底盘控制权
                self.move_base.cancel_all_goals()
                rospy.sleep(0.5) # 给底层一点反应时间
                
                try:
                    is_going_up = (target_floor > self.curr_floor) # 判断上楼还是下楼
                    action_str = "上楼" if is_going_up else "下楼"
                    
                    # ================= 【新增 4：爬楼前准备（加锁 + 挂起HDL）】 =================
                    self.is_climbing = True
                    if self.pause_hdl_srv:
                        rospy.loginfo("【联动】正在暂停 HDL Governor，准备脱离地图盲爬...")
                        self.pause_hdl_srv(True)
                    # =========================================================================

                    rospy.loginfo(f"【联动】呼叫底层控制，执行 {action_str} 动作...")
                    
                    # 阻塞调用爬楼梯服务
                    if hasattr(self, 'climb_srv'):
                        resp = self.climb_srv(is_going_up)
                        
                        if resp.success:
                            rospy.loginfo(f"【联动】{action_str}动作已确认完成，狗已平稳站上目标楼层！")
                            self.curr_floor = target_floor  # 强行覆盖楼层认知，不再怕定时器捣乱
                        else:
                            rospy.logerr(f"【联动】{action_str}失败或被中断！")
                    else:
                        rospy.logerr("【错误】爬楼服务未初始化...")
                        
                except Exception as e:
                    rospy.logerr(f"调用爬楼服务发生异常: {e}")
                
                # 2. 爬楼完成后，执行 2D 导航图的平滑切换
                self.switch_map(target_floor)
                
                # ================= 【新增 5：爬楼后恢复（解锁 + 唤醒HDL）】 =================
                if self.pause_hdl_srv:
                    rospy.loginfo("【联动】新楼层地图就绪，恢复 HDL Governor 运行...")
                    self.pause_hdl_srv(False)
                
                self.is_climbing = False # 解锁，重新允许 TF 更新高度认知
                # =========================================================================
                
                # 3. 继续导航去最终目标点
                rospy.sleep(1.0)
                rospy.loginfo(f"Floor Manager: 换层流程结束，前往最终目标: {target_key}")
                self.send_goal(target_cfg[target_key])
                
            else:
                rospy.logerr("Floor Manager: 无法到达当前楼层的楼梯口，跨楼层任务中止。")
                
        else:
            # 目标层就在当前层
            rospy.loginfo(f"Floor Manager: 目标层即为当前层，直接前往目标点。")
            self.send_goal(target_cfg[target_key])

if __name__ == '__main__':
    try:
        mgr = FloorManager()
        rospy.sleep(2.0)
        target_f = rospy.get_param('~initial_target_floor', 4)
        target_k = rospy.get_param('~initial_target_key', 'office_goal')
        mgr.run_mission(target_f, target_k)
    except rospy.ROSInterruptException:
        pass