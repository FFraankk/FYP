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
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'aft_mapped') # 强依赖 FAST-LIO 高精度里程计
        
        # 你的狗爬楼梯时设定的线速度，根据实际情况微调
        self.climb_speed = rospy.get_param('~climb_speed', 0.2) 
        
        # 楼层高度差参数 (单位: 米) -> 请根据你实际大楼的楼梯去卷尺量一下
        self.HALF_HEIGHT = rospy.get_param('~half_height', 1.5) # 半层楼（到达休息平台）的高度
        self.FULL_HEIGHT = rospy.get_param('~full_height', 3.0) # 整层楼的高度差
        
        # 楼梯旋向：1.0 为左旋(上楼时左转)，-1.0 为右旋(上楼时右转)
        self.stair_turn_direction = rospy.get_param('~turn_direction', 1.0) 
        
        # --- ROS 接口 ---
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        
        # 声明接管爬楼梯的服务
        self.srv = rospy.Service('/execute_stair_climb', SetBool, self.handle_climb)
        rospy.loginfo("【StairClimber】独立爬楼控制节点已启动，挂载服务 /execute_stair_climb，等待调遣...")

    def handle_climb(self, req):
        is_up = req.data
        direction_str = "上楼" if is_up else "下楼"
        rospy.loginfo(f"【StairClimber】接到 {direction_str} 指令，开始双跑(U型)楼梯机动...")
        
        rate = rospy.Rate(10)
        start_yaw = None
        start_z = None
        
        # 1. 锁定初始起步状态
        while start_yaw is None and not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                euler = tft.euler_from_quaternion(rot)
                start_yaw = euler[2]
                start_z = trans[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rate.sleep()

        rospy.loginfo(f"【StairClimber】起步锁定 -> 初始航向: {start_yaw:.2f} rad, 初始高度: {start_z:.2f} m")

        # 2. 状态机初始化
        state = 1
        target_yaw = start_yaw
        cmd = Twist()
        
        # 3. 核心控制大循环
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                euler = tft.euler_from_quaternion(rot)
                curr_yaw = euler[2]
                curr_pitch = euler[1]
                curr_z = trans[2]
            except:
                continue
                
            # 计算已经爬升/下降的绝对高度差
            height_diff = abs(curr_z - start_z)

            # ================= 状态 1：攀爬上半段 =================
            if state == 1:
                # 触发条件：高度接近半层，且狗的身体恢复水平（Pitch接近0）
                if height_diff > (self.HALF_HEIGHT - 0.3) and abs(curr_pitch) < 0.05:
                    rospy.loginfo(f"【StairClimber】检测到平台(当前高度差:{height_diff:.2f}m)，准备掉头机动。")
                    self.stop_robot()
                    state = 2
                    continue # 立即进入下一次循环执行 state 2
                
                # 动作：盲走并锁定航向
                cmd.linear.x = self.climb_speed
                cmd.angular.z = 0.5 * normalize_angle(target_yaw - curr_yaw)
                self.cmd_pub.publish(cmd)

            # ================= 状态 2：休息平台机动 =================
            elif state == 2:
                # 这是一个阻塞式的多段机动动作
                self.execute_landing_maneuver(start_yaw, is_up)
                
                # 机动完成后，狗已经掉头180度，更新大循环的目标航向
                target_yaw = normalize_angle(start_yaw + math.pi)
                
                rospy.loginfo("【StairClimber】机动完成，已对齐第二段台阶，继续攀爬...")
                state = 3

            # ================= 状态 3：攀爬下半段 =================
            elif state == 3:
                # 触发条件：高度接近整层，且狗的身体恢复水平
                if height_diff > (self.FULL_HEIGHT - 0.4) and abs(curr_pitch) < 0.05:
                    rospy.loginfo(f"【StairClimber】检测到最终楼层平地(总高度差:{height_diff:.2f}m)！任务成功。")
                    self.stop_robot()
                    break # 彻底跳出循环，爬楼结束
                
                # 动作：以反向航向盲走
                cmd.linear.x = self.climb_speed
                cmd.angular.z = 0.5 * normalize_angle(target_yaw - curr_yaw)
                self.cmd_pub.publish(cmd)

            rate.sleep()

        return SetBoolResponse(success=True, message=f"{direction_str}机动完美结束")

    # ================= 平台机动逻辑 =================
    def execute_landing_maneuver(self, start_yaw, is_up):
        """
        处理休息平台的 U 型掉头：前进 -> 转90度 -> 前进 -> 转90度 -> 前进贴近
        """
        # 上楼和下楼的转弯方向是相反的
        turn_sign = self.stair_turn_direction if is_up else -self.stair_turn_direction
        
        # [动作 1] 往前走，确保狗的后腿完全脱离台阶，彻底站上平台
        rospy.loginfo(">> 动作1: 前进，完全站上平台...")
        self.move_forward(duration=2.5, speed=self.climb_speed) 
        
        # [动作 2] 原地旋转 90 度
        rospy.loginfo(">> 动作2: 第一次闭环旋转 90 度...")
        target_yaw_1 = normalize_angle(start_yaw + turn_sign * (math.pi / 2.0))
        self.turn_to_yaw(target_yaw_1)
        
        # [动作 3] 往前走，跨越休息平台的宽度
        rospy.loginfo(">> 动作3: 前进，跨越平台宽度...")
        # 这里的 duration 根据你平台的大小调整，平台越宽走越久
        self.move_forward(duration=3.5, speed=self.climb_speed) 
        
        # [动作 4] 再原地旋转 90 度，完成 180 度掉头
        rospy.loginfo(">> 动作4: 第二次闭环旋转 90 度，对准新台阶...")
        target_yaw_2 = normalize_angle(start_yaw + turn_sign * math.pi)
        self.turn_to_yaw(target_yaw_2)
        
        # [动作 5] 往前走一小步，让狗的前腿摸到第二段台阶的边缘
        rospy.loginfo(">> 动作5: 贴近下一段台阶...")
        self.move_forward(duration=1.5, speed=self.climb_speed)
        
        rospy.loginfo("【StairClimber】休息平台多段机动执行完毕。")

    # ================= 底层封装动作 =================
    def stop_robot(self):
        """急刹车并等待站稳"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        rospy.sleep(1.0) 

    def move_forward(self, duration, speed=0.2):
        """开环控制：以固定速度前进指定时间"""
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0
        
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_pub.publish(cmd)
            rate.sleep()
            
        self.stop_robot()

    def turn_to_yaw(self, target_yaw):
        """闭环控制：利用 FAST-LIO TF 原地旋转到绝对目标航向"""
        cmd = Twist()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            try:
                (_, rot) = self.tf_listener.lookupTransform(self.map_frame, self.odom_frame, rospy.Time(0))
                curr_yaw = tft.euler_from_quaternion(rot)[2]
            except: 
                continue
            
            error = normalize_angle(target_yaw - curr_yaw)
            
            # 误差小于 0.08 rad (约 4.5 度) 即认为对齐完成
            if abs(error) < 0.08: 
                break
                
            cmd.linear.x = 0.0
            
            # P 控制器，系数 0.6
            cmd.angular.z = 0.6 * error 
            # 限制最大角速度，防止急转弯打滑或侧翻
            cmd.angular.z = max(min(cmd.angular.z, 0.4), -0.4) 
            
            self.cmd_pub.publish(cmd)
            rate.sleep()
            
        self.stop_robot()

if __name__ == '__main__':
    try:
        StairClimber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass