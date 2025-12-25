#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import subprocess


class RobotFSM:
    def __init__(self):
        self.state = "IDLE"

        self.state_pub = rospy.Publisher("/fsm_state", String, queue_size=10)
        rospy.Subscriber("/fsm_cmd", String, self.cmd_callback)

        rospy.Timer(rospy.Duration(1.0), self.tick)

        self.move_base_proc = None

    def cmd_callback(self, msg):
        cmd = msg.data.strip()

        if cmd == "NAV_FLAT" and self.state != "NAV_FLAT":
            rospy.loginfo("FSM: start navigation")
            self.start_navigation()
            self.state = "NAV_FLAT"

        elif cmd == "STOP" and self.state != "IDLE":
            rospy.loginfo("FSM: stop navigation")
            self.stop_navigation()
            self.state = "IDLE"

    def tick(self, event):
        rospy.loginfo("FSM state: %s", self.state)
        self.state_pub.publish(self.state)
        
    def start_navigation(self):
        if self.move_base_proc is None:
            self.move_base_proc = subprocess.Popen(
                ["roslaunch", "robot_bringup", "navigation.launch"]
            )

    def stop_navigation(self):
        if self.move_base_proc is not None:
            self.move_base_proc.terminate()
            self.move_base_proc = None

if __name__ == "__main__":
    rospy.init_node("robot_fsm")
    RobotFSM()
    rospy.spin()
