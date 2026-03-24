#!/usr/bin/env python3

import rospy
import sys
import time
from geometry_msgs.msg import Twist

# --- Unitree SDK 核心与运动控制导入 ---
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.go2.sport.sport_client import SportClient

# --- 新增：雷达控制所需的 DDS 消息导入 ---
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
from unitree_sdk2py.idl.default import std_msgs_msg_dds__String_


class Go2SafeBridge:
    def __init__(self, interface):
        # 0. 必须最先初始化 ROS 节点！
        rospy.init_node('go2_ros_bridge_safe', anonymous=True)
        
        # --- 参数配置 (可根据环境调整) ---
        self.MAX_LINEAR_X = 0.8   # 最大前进速度 (m/s)
        self.MAX_LINEAR_Y = 0.8   # 最大侧移速度 (m/s)
        self.MAX_ANGULAR_Z = 1.0  # 最大旋转速度 (rad/s)
        self.TIMEOUT_SEC = 0.5    # 超过 0.5s 没收到指令就停机
        
        # 1. 初始化 Unitree SDK
        ChannelFactoryInitialize(0, interface)
        self.sport_client = SportClient()
        self.sport_client.SetTimeout(10.0)
        self.sport_client.Init()
        
        # 【新增】初始化雷达控制 Publisher
        self.lidar_pub = ChannelPublisher("rt/utlidar/switch", String_)
        self.lidar_pub.Init()
        
        # 2. 初始化状态控制
        self.last_cmd_time = rospy.Time.now() # 初始化为当前时间
        self.is_stopped = True

        # --- 启动序列控制 ---
        rospy.loginfo("Initializing Go2 Sequence...")
        
        # 【新增步骤 A】：先关闭雷达
        rospy.loginfo("Step 1: Turning off LiDAR...")
        self.switch_lidar("OFF")
        time.sleep(1.0) # 稍微延时，确保 DDS 指令发送并生效
        
        # 【新增步骤 B】：坐下/趴下进入阻尼模式
        rospy.loginfo("Step 2: Dog is sitting down (StandDown)...")
        self.sport_client.StandDown()
        time.sleep(2.0)
        
        rospy.loginfo("Step 3: Sending Recovery Stand...")
        self.sport_client.RecoveryStand()
        time.sleep(3.0) 
        
        rospy.loginfo("Step 4: Switching to Free Walk mode...")
        ret = self.sport_client.FreeWalk()
        if ret == 0:
            rospy.loginfo("Successfully switched to Free Walk.")
        else:
            rospy.logwarn(f"Failed to switch to Free Walk! Return code: {ret}")
        # ==========================================================
        
        # 3. 订阅 /cmd_vel 话题
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.loginfo("Successfully subscribed to /cmd_vel.")
        
        # 4. 启动安全检查定时器 (20Hz)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.safety_watchdog)
        rospy.loginfo("Safety Watchdog timer started.")
        
    def switch_lidar(self, status):
        """【新增】控制雷达开关的辅助方法"""
        cmd_msg = std_msgs_msg_dds__String_()
        if status == "OFF":
            cmd_msg.data = "OFF"
        elif status == "ON":
            cmd_msg.data = "ON"
        
        self.lidar_pub.Write(cmd_msg)
        rospy.loginfo(f"LiDAR switch command sent: {status}")

    def cmd_vel_callback(self, msg):
        # 记录收到指令的时间戳
        self.last_cmd_time = rospy.Time.now()
        
        # --- 速度限幅保护 ---
        vx = max(min(msg.linear.x, self.MAX_LINEAR_X), -self.MAX_LINEAR_X)
        vy = max(min(msg.linear.y, self.MAX_LINEAR_Y), -self.MAX_LINEAR_Y)
        vyaw = max(min(msg.angular.z, self.MAX_ANGULAR_Z), -self.MAX_ANGULAR_Z)
        
        try:
            self.sport_client.Move(vx, vy, vyaw)
            self.is_stopped = False
        except Exception as e:
            rospy.logerr(f"SDK Move Error: {e}")

    def safety_watchdog(self, event):
        """
        心跳检测定时器：如果超过规定时间没收到新指令，强制停止机器人。
        """
        if self.is_stopped:
            return

        time_since_last_cmd = (rospy.Time.now() - self.last_cmd_time).to_sec()
        
        if time_since_last_cmd > self.TIMEOUT_SEC:
            rospy.logwarn(f"WATCHDOG: No cmd_vel received for {time_since_last_cmd:.2f}s! Emergency Stop.")
            try:
                self.sport_client.StopMove()
                # 连续发送几次零速确保指令到达
                self.sport_client.Move(0, 0, 0)
                self.is_stopped = True
            except Exception as e:
                rospy.logerr(f"Watchdog stop failed: {e}")

    def run(self):
        rospy.on_shutdown(self.shutdown_handle)
        rospy.spin()

    def shutdown_handle(self):
        rospy.loginfo("Node shutting down. Resetting dog state.")
        try:
            self.sport_client.StopMove()
            # 可以在节点关闭时也确保它坐下
            # self.sport_client.StandDown() 
            # 如果退出时需要重新打开雷达，可以解除下方注释：
            # self.switch_lidar("ON") 
        except:
            pass

if __name__ == '__main__':
    net_interface = "eth1"
    if len(sys.argv) > 1 and not sys.argv[1].startswith('__'):
        net_interface = sys.argv[1]

    try:
        bridge = Go2SafeBridge(net_interface)
        bridge.run()
    except rospy.ROSInterruptException:
        pass