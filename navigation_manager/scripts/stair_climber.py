#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# # 上楼测试

# rosservice call /execute_stair_climb "data: true"



# # 下楼测试

# rosservice call /execute_stair_climb "data: false"

import rospy
import tf
import math
import tf.transformations as tft
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

# ================= 辅助函数 =================
def normalize_angle(angle):
    """将角度归一化到 -pi 到 pi 之间"""
    return math.atan2(math.sin(angle), math.cos(angle))

# ================= 核心类 =================
class StairClimber:
    def __init__(self):
        rospy.init_node('stair_climber_node')
        
        # --- 参数配置 ---
        self.map_frame = rospy.get_param('~map_frame', 'camera_init')
        self.odom_frame = rospy.get_param('~odom_frame', 'aft_mapped') 
        self.climb_speed = rospy.get_param('~climb_speed', 0.4) 
        self.HALF_HEIGHT = rospy.get_param('~half_height', 2.14) 
        self.FULL_HEIGHT = rospy.get_param('~full_height', 4.28) 
        self.stair_turn_direction = rospy.get_param('~turn_direction', -1.0) 
        
        # --- ROS 接口 ---
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.srv = rospy.Service('/execute_stair_climb', SetBool, self.handle_climb)
        
        rospy.loginfo("="*50)
        rospy.loginfo("【StairClimber】节点启动成功")
        rospy.loginfo(f"配置参数: 半层高度={self.HALF_HEIGHT}m, 全层高度={self.FULL_HEIGHT}m")
        rospy.loginfo(f"转弯方向: {'左旋' if self.stair_turn_direction > 0 else '右旋'}")
        rospy.loginfo("="*50)

    def handle_climb(self, req):
        is_up = req.data
        direction_str = "上楼" if is_up else "下楼"
        rospy.loginfo(f"🚀 >>> 接收到指令: 【{direction_str}】 任务开始执行...")
        
        rate = rospy.Rate(10)
        start_yaw = None
        start_z = None
        
        rospy.loginfo("正在同步 TF 坐标系，尝试获取起始位姿...")
        while start_yaw is None and not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                euler = tft.euler_from_quaternion(rot)
                start_yaw = euler[2]
                start_z = trans[2]
            except Exception as e:
                rospy.logwarn_throttle(2, f"等待 TF 数据中 ({self.map_frame} -> {self.odom_frame})...")
                rate.sleep()

        rospy.loginfo(f"✅ 起步位姿锁定 -> Z: {start_z:.2f}m, Yaw: {start_yaw:.2f}rad")

        # 2. 状态机初始化
        state = 1
        target_yaw = start_yaw
        cmd = Twist()
        log_counter = 0 # 用于限制循环内日志频率
        
        # 3. 核心控制大循环
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                euler = tft.euler_from_quaternion(rot)
                curr_yaw, curr_pitch, curr_z = euler[2], euler[1], trans[2]
            except:
                continue
                
            height_diff = abs(curr_z - start_z)
            yaw_err = normalize_angle(target_yaw - curr_yaw)

            # 每秒打印一次实时反馈，方便调试观察触发条件
            log_counter += 1
            if log_counter % 10 == 0:
                rospy.loginfo(f"[STATE {state}] 高度差: {height_diff:.2f}/{self.HALF_HEIGHT if state<3 else self.FULL_HEIGHT:.2f} | Pitch: {curr_pitch:.2f} | Yaw误差: {yaw_err:.2f}")

            # ================= 状态 1：攀爬上半段 =================
            if state == 1:
                if height_diff > (self.HALF_HEIGHT - 0.3) and abs(curr_pitch) < 0.05:
                    rospy.loginfo("🎯 [STEP 1 完成] 检测到休息平台，停止并准备掉头。")
                    self.stop_robot()
                    state = 2
                    continue 
                
                cmd.linear.x = self.climb_speed
                cmd.angular.z = 0.5 * yaw_err
                self.cmd_pub.publish(cmd)

            # ================= 状态 2：休息平台机动 =================
            elif state == 2:
                rospy.loginfo("🚧 [STEP 2] 进入 U 型平台转弯机动...")
                self.execute_landing_maneuver(start_yaw, is_up)
                target_yaw = normalize_angle(start_yaw + math.pi)
                rospy.loginfo(f"🔄 平台机动结束。新目标航向: {target_yaw:.2f} rad")
                state = 3

            # ================= 状态 3：攀爬下半段 =================
            elif state == 3:
                if height_diff > (self.FULL_HEIGHT - 0.4) and abs(curr_pitch) < 0.05:
                    rospy.loginfo(f"🏁 [STEP 3 完成] 到达目标楼层！最终高度差: {height_diff:.2f}m")
                    self.stop_robot()
                    break 
                
                cmd.linear.x = self.climb_speed
                cmd.angular.z = 0.5 * yaw_err
                self.cmd_pub.publish(cmd)

            rate.sleep()

        return SetBoolResponse(success=True, message=f"{direction_str}机动执行完毕")

    # ================= 平台机动逻辑 =================
    def execute_landing_maneuver(self, start_yaw, is_up):
        turn_sign = self.stair_turn_direction if is_up else -self.stair_turn_direction
        
        rospy.loginfo(">>> 动作1: 前进 2.5s，确保后腿站稳平台...")
        self.move_forward(duration=1.0, speed=self.climb_speed) 
        
        rospy.loginfo(">>> 动作2: 第一次 90° 旋转...")
        target_yaw_1 = normalize_angle(start_yaw + turn_sign * (math.pi / 1.8))
        self.turn_to_yaw(target_yaw_1)
        
        rospy.loginfo(">>> 动作3: 横穿平台 3.5s...")
        self.move_forward(duration=3.5, speed=self.climb_speed) 
        
        rospy.loginfo(">>> 动作4: 第二次 90° 旋转，对准下一段...")
        target_yaw_2 = normalize_angle(start_yaw + turn_sign * math.pi)
        self.turn_to_yaw(target_yaw_2)
        
        rospy.loginfo(">>> 动作5: 最后微调前进 1.5s，贴近楼梯边缘...")
        self.move_forward(duration=1.5, speed=self.climb_speed)

    # ================= 底层封装动作 =================
    def stop_robot(self):
        rospy.logdebug("停止运动并等待 1s...")
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rospy.sleep(1.0) 

    def move_forward(self, duration, speed=0.2):
        cmd = Twist()
        cmd.linear.x = speed
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= duration:
                break
            self.cmd_pub.publish(cmd)
            rate.sleep()
            
        self.stop_robot()

    def turn_to_yaw(self, target_yaw):
            rospy.loginfo(f"   [旋转中] 目标: {target_yaw:.2f} rad")
            cmd = Twist()
            rate = rospy.Rate(10)
            t_start = rospy.Time.now()
            
            # --- 关键调试参数 ---
            min_ang_vel = 0.4    
            max_ang_vel = 0.6    
            p_gain = 0.8         
            tolerance = 0.06     
            # ------------------

            while not rospy.is_shutdown():
                try:
                    # 获取当前位姿
                    (_, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                    curr_yaw = tft.euler_from_quaternion(rot)[2]
                except: 
                    continue
                
                # 计算残余误差
                error = normalize_angle(target_yaw - curr_yaw)
                
                # 检查是否到达目标
                if abs(error) < tolerance: 
                    t_end = rospy.Time.now()
                    rospy.loginfo(f"   ✅ [旋转完成] 耗时: {(t_end-t_start).to_sec():.2f}s, 残留误差: {error:.4f}")
                    break
                    
                # 1. 计算基础 P 输出
                target_vel = p_gain * error 
                
                # 2. 克服死区：如果速度太小，强制提升到 min_ang_vel，但保持符号（方向）一致
                if abs(target_vel) < min_ang_vel:
                    target_vel = math.copysign(min_ang_vel, error)
                
                # 3. 限制物理最大速度
                cmd.angular.z = max(min(target_vel, max_ang_vel), -max_ang_vel) 
                
                # 打印调试信息（可选，如果嫌 log 太多可以注释掉）
                # rospy.loginfo_throttle(0.5, f"      实时误差: {error:.3f} | 输出角速度: {cmd.angular.z:.3f}")
                
                self.cmd_pub.publish(cmd)
                rate.sleep()
                
            # 彻底停止
            self.stop_robot()
            
if __name__ == '__main__':
    try:
        node = StairClimber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass