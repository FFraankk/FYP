#!/usr/bin/env python3

import rospy
import sys
import time
from geometry_msgs.msg import Twist
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

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
                
                # 2. 初始化状态控制
                self.last_cmd_time = rospy.Time.now() # 初始化为当前时间
                self.is_stopped = True

                # --- 更强壮的模式切换逻辑 ---
                rospy.loginfo("Initializing Go2 Sport Mode...")
                
                # 先解除异常状态，进入阻尼模式
                self.sport_client.Damp()
                time.sleep(0.5)
                
                # 恢复站立
                rospy.loginfo("Sending Recovery Stand...")
                self.sport_client.RecoveryStand()
                time.sleep(3.0) 
                
                # 切换到 FreeWalk 模式
                rospy.loginfo("Switching to Free Walk mode...")
                ret = self.sport_client.FreeWalk()
                if ret == 0:
                    rospy.loginfo("Successfully switched to Free Walk.")
                else:
                    rospy.logwarn(f"Failed to switch to Free Walk! Return code: {ret}")
                # ------------------
                
                # ================= 丢失的代码补回 =================
                # 3. 订阅 /cmd_vel 话题
                self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
                rospy.loginfo("Successfully subscribed to /cmd_vel.")
                
                # 4. 启动安全检查定时器 (20Hz)
                self.timer = rospy.Timer(rospy.Duration(0.05), self.safety_watchdog)
                rospy.loginfo("Safety Watchdog timer started.")
                # ==================================================
            
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
            # 可以在这里加一个让狗坐下的动作，或者保持平衡站立
            # self.sport_client.StandDown() 
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