#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from collections import deque

import actionlib
import rospkg
import rospy
import tf
import yaml
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32
from std_srvs.srv import Empty, SetBool


class FloorManager:
    def __init__(self):
        rospy.init_node("floor_manager_node")

        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "camera_init")
        self.odom_frame = rospy.get_param("~odom_frame", "aft_mapped")
        self.move_base_ns = rospy.get_param("~move_base_ns", "move_base")
        self.curr_floor = rospy.get_param("~initial_floor", None)

        self.rp = rospkg.RosPack()
        self.package_path = self.rp.get_path("navigation_manager")
        self.load_config()

        if self.curr_floor is None:
            self.curr_floor = self.default_initial_floor

        self.curr_floor = int(self.curr_floor)
        self.curr_pos = None
        self.is_climbing = False

        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient(self.move_base_ns, MoveBaseAction)

        rospy.loginfo("Floor Manager: waiting for move_base...")
        if not self.move_base.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Floor Manager: move_base server is unavailable")

        try:
            self.clear_srv = rospy.ServiceProxy(f"/{self.move_base_ns}/clear_costmaps", Empty)
        except Exception as exc:
            rospy.logwarn(f"Floor Manager: clear_costmaps is unavailable: {exc}")
            self.clear_srv = None

        self.z_offset_pub = rospy.Publisher("/set_z_offset", Float32, queue_size=1)

        try:
            rospy.wait_for_service("/execute_stair_climb", timeout=5.0)
            self.climb_srv = rospy.ServiceProxy("/execute_stair_climb", SetBool)
        except rospy.ROSException:
            self.climb_srv = None
            rospy.logwarn("Floor Manager: /execute_stair_climb is unavailable")

        try:
            rospy.wait_for_service("/set_hdl_pause", timeout=3.0)
            self.pause_hdl_srv = rospy.ServiceProxy("/set_hdl_pause", SetBool)
        except rospy.ROSException:
            self.pause_hdl_srv = None
            rospy.logwarn("Floor Manager: /set_hdl_pause is unavailable")

        rospy.Timer(rospy.Duration(0.1), self.update_pose_from_tf)
        rospy.loginfo(
            f"Floor Manager: ready, current floor={self.curr_floor}, "
            f"known floors={sorted(self.floors.keys())}"
        )

    def load_config(self):
        config_path = os.path.join(self.package_path, "config", "waypoints.yaml")
        try:
            with open(config_path, "r", encoding="utf-8") as handle:
                cfg = yaml.safe_load(handle) or {}
        except Exception as exc:
            rospy.logerr(f"Floor Manager: failed to load config: {exc}")
            rospy.signal_shutdown("config load failure")
            return

        raw_floors = cfg.get("floors", {})
        raw_edges = cfg.get("stair_edges", [])
        settings = cfg.get("settings", {})

        self.floors = {int(key): value for key, value in raw_floors.items()}
        self.stair_edges = []
        for edge in raw_edges:
            parsed = dict(edge)
            parsed["from_floor"] = int(parsed["from_floor"])
            parsed["to_floor"] = int(parsed["to_floor"])
            self.stair_edges.append(parsed)

        self.default_initial_floor = int(settings.get("initial_floor", 0))
        self.floor_z_offsets = {
            int(key): float(value)
            for key, value in (settings.get("floor_z_offsets", {}) or {}).items()
        }
        self.floor_thresholds = {
            int(key): float(value)
            for key, value in (settings.get("floor_thresholds", {}) or {}).items()
        }

        if not self.floors:
            raise RuntimeError("waypoints.yaml must define 'floors'")

    def update_pose_from_tf(self, _event):
        if self.is_climbing:
            return

        try:
            if self.tf_listener.canTransform(self.map_frame, self.odom_frame, rospy.Time(0)):
                trans, _rot = self.tf_listener.lookupTransform(
                    self.map_frame, self.odom_frame, rospy.Time(0)
                )
                self.curr_pos = trans

                if self.curr_floor is None and self.floor_thresholds:
                    self.curr_floor = self.estimate_floor_from_height(trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(10, f"Floor Manager: TF exception: {exc}")

    def estimate_floor_from_height(self, z_value):
        ordered = sorted(self.floor_thresholds.items(), key=lambda item: item[1])
        estimated = ordered[0][0]
        for floor_id, threshold in ordered:
            if z_value >= threshold:
                estimated = floor_id
        return estimated

    def get_floor_cfg(self, floor_id):
        if floor_id not in self.floors:
            raise KeyError(f"floor {floor_id} is not configured")
        return self.floors[floor_id]

    def get_pose(self, floor_id, group_name, point_name):
        floor_cfg = self.get_floor_cfg(floor_id)
        group = floor_cfg.get(group_name, {})
        if point_name not in group:
            raise KeyError(f"{group_name}.{point_name} is missing on floor {floor_id}")
        return group[point_name]

    def switch_map(self, floor_id):
        floor_cfg = self.get_floor_cfg(floor_id)
        map_name = floor_cfg["map_name"]
        map_path = os.path.join(self.package_path, "maps", map_name)

        rospy.loginfo(f"Floor Manager: switching map to floor {floor_id} -> {map_name}")
        self.move_base.cancel_all_goals()
        os.system("rosnode kill /map_server")
        os.system("rosnode kill /move_base")
        rospy.sleep(3.0)
        os.system(f"rosrun map_server map_server {map_path} __name:=map_server &")

        rospy.loginfo("Floor Manager: waiting for move_base restart...")
        if not self.move_base.wait_for_server(rospy.Duration(15.0)):
            rospy.logerr("Floor Manager: move_base restart failed")

    def publish_floor_offset(self, floor_id, edge=None):
        if edge and "z_offset" in edge:
            offset = float(edge["z_offset"])
        else:
            offset = float(self.floor_z_offsets.get(floor_id, 0.0))

        rospy.loginfo(f"Floor Manager: publish z offset {offset:.3f} for floor {floor_id}")
        self.z_offset_pub.publish(Float32(offset))

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z = pose[0:3]
        (
            goal.target_pose.pose.orientation.x,
            goal.target_pose.pose.orientation.y,
            goal.target_pose.pose.orientation.z,
            goal.target_pose.pose.orientation.w,
        ) = pose[3:7]

        rospy.loginfo("Floor Manager: sending move_base goal")
        self.move_base.send_goal(goal)
        self.move_base.wait_for_result()
        return self.move_base.get_state()

    def find_route(self, start_floor, target_floor):
        if start_floor == target_floor:
            return []

        queue = deque([(start_floor, [])])
        visited = {start_floor}

        while queue:
            floor_id, route = queue.popleft()
            for edge in self.stair_edges:
                if edge["from_floor"] != floor_id:
                    continue

                next_floor = edge["to_floor"]
                if next_floor in visited:
                    continue

                next_route = route + [edge]
                if next_floor == target_floor:
                    return next_route

                visited.add(next_floor)
                queue.append((next_floor, next_route))

        return None

    def execute_edge(self, edge):
        from_floor = edge["from_floor"]
        to_floor = edge["to_floor"]
        from_stair = edge["from_stair"]
        to_stair = edge.get("to_stair", "")
        is_going_up = edge.get("is_going_up")
        if is_going_up is None:
            is_going_up = to_floor > from_floor

        if self.curr_floor != from_floor:
            rospy.logerr(
                f"Floor Manager: current floor {self.curr_floor} does not match "
                f"edge start floor {from_floor}"
            )
            return False

        start_pose = self.get_pose(from_floor, "stairs", from_stair)
        rospy.loginfo(
            f"Floor Manager: go to floor {from_floor} stair {from_stair}, "
            f"then climb to floor {to_floor} stair {to_stair}"
        )

        if self.send_goal(start_pose) != GoalStatus.SUCCEEDED:
            rospy.logerr(f"Floor Manager: failed to reach stair {from_stair} on floor {from_floor}")
            return False

        self.move_base.cancel_all_goals()
        rospy.sleep(0.5)

        if self.pause_hdl_srv:
            try:
                self.pause_hdl_srv(True)
            except Exception as exc:
                rospy.logwarn(f"Floor Manager: failed to pause HDL: {exc}")

        if not self.climb_srv:
            rospy.logerr("Floor Manager: climb service is unavailable")
            return False

        try:
            self.is_climbing = True
            resp = self.climb_srv(bool(is_going_up))
        except Exception as exc:
            rospy.logerr(f"Floor Manager: climb service call failed: {exc}")
            self.is_climbing = False
            return False

        if not resp.success:
            rospy.logerr(f"Floor Manager: climb failed: {resp.message}")
            self.is_climbing = False
            return False

        self.curr_floor = to_floor
        self.publish_floor_offset(to_floor, edge)
        self.switch_map(to_floor)
        self.is_climbing = False
        rospy.sleep(1.0)
        return True

    def run_mission(self, target_floor, target_key):
        target_floor = int(target_floor)
        rospy.loginfo(
            f"Floor Manager: start mission current={self.curr_floor} "
            f"target_floor={target_floor} target_key={target_key}"
        )

        route = self.find_route(self.curr_floor, target_floor)
        if route is None:
            rospy.logerr(
                f"Floor Manager: no stair route found from floor {self.curr_floor} to {target_floor}"
            )
            return

        for edge in route:
            if rospy.is_shutdown():
                return
            if not self.execute_edge(edge):
                rospy.logerr("Floor Manager: mission aborted during stair traversal")
                return

        target_pose = self.get_pose(target_floor, "goals", target_key)
        rospy.loginfo(
            f"Floor Manager: reached target floor {target_floor}, "
            f"navigating to goal {target_key}"
        )
        self.send_goal(target_pose)


if __name__ == "__main__":
    try:
        manager = FloorManager()
        rospy.sleep(2.0)
        target_floor = rospy.get_param("~initial_target_floor", 0)
        target_key = rospy.get_param("~initial_target_key", "office_goal")
        manager.run_mission(target_floor, target_key)
    except rospy.ROSInterruptException:
        pass
