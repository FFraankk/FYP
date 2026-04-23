#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StairClimber:
    def __init__(self):
        rospy.init_node("stair_climber_node")

        self.map_frame = rospy.get_param("~map_frame", "camera_init")
        self.odom_frame = rospy.get_param("~odom_frame", "aft_mapped")
        self.climb_speed = rospy.get_param("~climb_speed", 0.4)
        self.half_height = rospy.get_param("~half_height", 2.14)
        self.full_height = rospy.get_param("~full_height", 4.28)
        self.platform_forward_duration = rospy.get_param("~platform_forward_duration", 1.5)
        self.top_exit_duration = rospy.get_param("~top_exit_duration", 6.0)
        self.pitch_flat_threshold = rospy.get_param("~pitch_flat_threshold", 0.05)
        self.half_height_margin = rospy.get_param("~half_height_margin", 0.3)
        self.full_height_margin = rospy.get_param("~full_height_margin", 0.4)
        self.yaw_gain = rospy.get_param("~yaw_gain", 0.5)

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.srv = rospy.Service("/execute_stair_climb", SetBool, self.handle_climb)

        rospy.loginfo("=" * 50)
        rospy.loginfo("StairClimber: straight stair mode ready")
        rospy.loginfo(
            f"half_height={self.half_height:.2f}, full_height={self.full_height:.2f}, "
            f"platform_forward_duration={self.platform_forward_duration:.2f}, "
            f"top_exit_duration={self.top_exit_duration:.2f}"
        )
        rospy.loginfo("=" * 50)

    def handle_climb(self, req):
        is_up = req.data
        direction = "up" if is_up else "down"
        rospy.loginfo(f"StairClimber: start {direction} stair mission")

        start_yaw = None
        start_z = None
        rate = rospy.Rate(10)

        while start_yaw is None and not rospy.is_shutdown():
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    self.map_frame, self.odom_frame, rospy.Time(0)
                )
                start_yaw = tft.euler_from_quaternion(rot)[2]
                start_z = trans[2]
            except Exception:
                rospy.logwarn_throttle(2, "StairClimber: waiting for TF...")
                rate.sleep()

        if rospy.is_shutdown():
            return SetBoolResponse(success=False, message="shutdown")

        rospy.loginfo(f"StairClimber: locked pose z={start_z:.2f}, yaw={start_yaw:.2f}")

        state = 1
        log_counter = 0

        while not rospy.is_shutdown():
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    self.map_frame, self.odom_frame, rospy.Time(0)
                )
                euler = tft.euler_from_quaternion(rot)
            except Exception:
                rate.sleep()
                continue

            curr_yaw = euler[2]
            curr_pitch = euler[1]
            curr_z = trans[2]
            height_diff = abs(curr_z - start_z)
            yaw_err = normalize_angle(start_yaw - curr_yaw)

            log_counter += 1
            if log_counter % 10 == 0:
                target_height = self.half_height if state in (1, 2) else self.full_height
                rospy.loginfo(
                    f"[STATE {state}] height={height_diff:.2f}/{target_height:.2f}, "
                    f"pitch={curr_pitch:.2f}, yaw_err={yaw_err:.2f}"
                )

            if state == 1:
                if (
                    height_diff > (self.half_height - self.half_height_margin)
                    and abs(curr_pitch) < self.pitch_flat_threshold
                ):
                    rospy.loginfo("StairClimber: reached the landing platform")
                    self.stop_robot()
                    state = 2
                    continue

                self.publish_climb_cmd(yaw_err)

            elif state == 2:
                rospy.loginfo("StairClimber: crossing the landing platform straight ahead")
                self.move_forward(self.platform_forward_duration, self.climb_speed)
                state = 3

            elif state == 3:
                if (
                    height_diff > (self.full_height - self.full_height_margin)
                    and abs(curr_pitch) < self.pitch_flat_threshold
                ):
                    rospy.loginfo("StairClimber: reached the target floor")
                    self.stop_robot()
                    state = 4
                    continue

                self.publish_climb_cmd(yaw_err)

            elif state == 4:
                rospy.loginfo("StairClimber: executing top exit straight ahead")
                self.move_forward(self.top_exit_duration, self.climb_speed)
                break

            rate.sleep()

        self.stop_robot()
        return SetBoolResponse(success=True, message=f"{direction} stair action completed")

    def publish_climb_cmd(self, yaw_err):
        cmd = Twist()
        cmd.linear.x = self.climb_speed
        cmd.angular.z = self.yaw_gain * yaw_err
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())
        rospy.sleep(1.0)

    def move_forward(self, duration, speed):
        cmd = Twist()
        cmd.linear.x = speed
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() >= duration:
                break
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop_robot()


if __name__ == "__main__":
    try:
        StairClimber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
