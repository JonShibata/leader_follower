#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
from std_msgs.msg import String


class Follower(Node):
    def __init__(self):
        super().__init__("follower")

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/follower/cmd_vel", 10)
        self.input_request_publisher_ = self.create_publisher(
            String, "input_request", 10
        )

        self.input_response_sub_ = self.create_subscription(
            String, "input_response", self.input_response_callback, 10
        )
        self.pose_sub_ = self.create_subscription(
            Pose, "follower/pose", self.pose_callback, 10
        )
        self.leader_pose_sub_ = self.create_subscription(
            Pose, "leader/pose", self.leader_pose_callback, 10
        )
        self.leader_path_complete_sub_ = self.create_subscription(
            Bool, "leader/path_complete", self.leader_path_complete_callback, 10
        )
        self.turtle1_pose_sub_ = self.create_subscription(
            Pose, "turtle1/pose", self.turtle1_pose_callback, 10
        )
        self.turtle2_pose_sub_ = self.create_subscription(
            Pose, "turtle2/pose", self.turtle2_pose_callback, 10
        )
        self.timer = self.create_timer(0.1, self.follow_path)
        self.leader_path = []

        self.spawn_x = 1.0
        self.spawn_y = 2.0
        self.spawn_theta = 0.0

        self.leader_pose = None
        self.turtle_poses = [None, None]
        self.current_pose = None
        self.path_complete = False
        self.path_index = 0

        self.error_integral_orientation = 0.0
        self.kp_orientation = 3.0
        self.ki_orientation = 1.5
        self.kff_orientation = 4.0

        self.linear_speed = 2.0

        self.distance_min_to_leader = 2.0
        self.distance_min_to_turtles = 1.0
        self.distance_for_next_path_point = 0.45
        self.distance_for_path_mitigation = 1.2

        self.waiting_for_input = False
        self.waiting_for_input_counter = 0
        self.waiting_for_input_mitigation_events = 10

    def pose_callback(self, msg):
        self.current_pose = msg

    def leader_pose_callback(self, msg):
        self.leader_path.append(msg)
        self.leader_pose = msg

    def turtle1_pose_callback(self, msg):
        self.turtle_poses[0] = msg

    def turtle2_pose_callback(self, msg):
        self.turtle_poses[1] = msg

    def leader_path_complete_callback(self, msg):
        self.path_complete = msg.data

    def topic_data_is_valid(self):
        return all(self.turtle_poses) and self.leader_pose and self.current_pose

    def spawn_follower(self):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting to spawn follower")

        request = Spawn.Request()
        request.x = self.spawn_x
        request.y = self.spawn_y
        request.theta = self.spawn_theta
        request.name = "follower"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Spawned follower")
        else:
            self.get_logger().error("Failed to spawn follower")

        request = SetPen.Request()
        request.r = 255
        request.g = 0
        request.b = 0
        set_pen_client = self.create_client(SetPen, "follower/set_pen")
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting to set pen")
        future = set_pen_client.call_async(request)

    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2.0 + (point1.y - point2.y) ** 2.0)

    def wrap_to_pi(self, angle):
        if angle > math.pi:
            angle -= 2.0 * math.pi
        if angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calc_orientation(self, point1, point2):
        return math.atan2(
            point1.y - point2.y,
            point1.x - point2.x,
        )

    def find_next_point(self):
        found_next_point = False
        while not found_next_point:
            if (
                self.distance_between_points(
                    self.current_pose, self.leader_path[self.path_index]
                )
                < self.distance_for_next_path_point
            ):
                self.path_index += 1
                if self.path_index >= len(self.leader_path):
                    found_next_point = True
            else:
                found_next_point = True

    def control_direction(self):

        desired_orientation = self.calc_orientation(
            self.leader_path[self.path_index], self.current_pose
        )

        error = self.wrap_to_pi(desired_orientation - self.current_pose.theta)

        desired_orientation_feedforward = self.kff_orientation * error

        self.error_integral_orientation += error

        return (
            (self.kp_orientation * error)
            + (self.ki_orientation * self.error_integral_orientation)
            + desired_orientation_feedforward
        )

    def move_to_point(self, twist):
        self.find_next_point()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.control_direction()

    def slow_backup(self, twist):
        twist.linear.x = -0.1
        twist.angular.z = 0.0

    def stop_motion(self, twist):
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    def calc_obstruction_angle(self, obstruction_pose):
        return self.wrap_to_pi(
            self.calc_orientation(obstruction_pose, self.current_pose)
            - self.current_pose.theta
        )

    def mitigate_obstruction(self, obstruction_angle):

        options = ["circle around"]

        # determine priority of 90 deg turn
        if abs(obstruction_angle) < math.pi / 4.0:
            insert_index = 0
        else:
            insert_index = 1

        self.get_logger().info(f"Obstruction angle: {obstruction_angle}")
        if obstruction_angle < 0.0:
            options.insert(insert_index, "turn right")
            options.insert(2, "turn left")
        else:
            options.insert(insert_index, "turn left")
            options.insert(2, "turn right")

        self.waiting_for_input = True
        msg = String()
        for option in options:
            msg.data += option + "\n"
        self.input_request_publisher_.publish(msg)

    def input_response_callback(self, msg):

        user_input = msg.data
        self.get_logger().info(f"User input: {user_input}")

        if user_input == "turn left":
            self.delete_path_by_obstruction()
            self.plan_path_around_obstruction("left")
        elif user_input == "turn right":
            self.delete_path_by_obstruction()
            self.plan_path_around_obstruction("right")
        elif user_input == "circle around":
            self.circle_path_around_obstruction()

        self.waiting_for_input = False
        twist = Twist()
        self.move_to_point(twist)

    def circle_path_around_obstruction(self, turtle_id):

        found_all_points = False
        path_index_mod = self.path_index
        while not found_all_points:
            if (
                self.distance_between_points(
                    self.turtle_poses[turtle_id], self.leader_path[path_index_mod]
                )
                < self.distance_for_path_mitigation
            ):
                angle_to_obstruction = self.calc_orientation(
                    self.leader_path[path_index_mod], self.turtle_poses[turtle_id]
                )
                self.leader_path[path_index_mod].x = (
                    self.turtle_poses[turtle_id].x
                    + math.cos(angle_to_obstruction) * self.distance_for_path_mitigation
                )
                self.leader_path[path_index_mod].y = (
                    self.turtle_poses[turtle_id].y
                    + math.sin(angle_to_obstruction) * self.distance_for_path_mitigation
                )
                path_index_mod += 1
            else:
                found_all_points = True

    def delete_path_by_obstruction(self, turtle_id):

        found_all_points = False
        while not found_all_points:
            if (
                self.distance_between_points(
                    self.turtle_poses[turtle_id], self.leader_path[self.path_index]
                )
                < self.distance_for_path_mitigation
            ):
                self.leader_path.pop(self.path_index)
            else:
                found_all_points = True

    def plan_path_around_obstruction(self, direction):

        if direction == "left":
            angle = math.pi / 2.0
        else:
            angle = -math.pi / 2.0

        new_angle = self.wrap_to_pi(angle + self.current_pose.theta)

        new_point1 = Pose()
        # Move 90 deg to the chosen direction
        new_point1.x = (
            self.current_pose.x
            + math.cos(new_angle) * self.distance_for_path_mitigation
        )
        new_point1.y = (
            self.current_pose.y
            + math.sin(new_angle) * self.distance_for_path_mitigation
        )
        self.leader_path.insert(self.path_index, new_point1)

        # move forward by twice the mitigation distance in parallel to the original heading
        new_point2 = Pose()
        new_point2.x = (
            new_point1.x
            + math.cos(self.current_pose.theta)
            * self.distance_for_path_mitigation
            * 2.0
        )
        new_point2.y = (
            new_point1.y
            + math.sin(self.current_pose.theta)
            * self.distance_for_path_mitigation
            * 2.0
        )
        self.leader_path.insert(self.path_index + 1, new_point2)

    def follow_path(self):
        twist = Twist()
        if self.waiting_for_input:
            if (
                self.waiting_for_input_counter
                < self.waiting_for_input_mitigation_events
            ):
                self.waiting_for_input_counter += 1
                self.slow_backup(twist)
        elif (
            self.topic_data_is_valid()
            and len(self.leader_path) > 0
            and self.path_index <= len(self.leader_path)
        ):
            distance_to_leader = self.distance_between_points(
                self.current_pose, self.leader_pose
            )
            if distance_to_leader < self.distance_min_to_leader:
                self.stop_motion(twist)
            else:
                distance_to_turtles = []
                for pose in self.turtle_poses:
                    distance_to_turtles.append(
                        self.distance_between_points(self.current_pose, pose)
                    )
                if all(x > self.distance_min_to_turtles for x in distance_to_turtles):
                    self.move_to_point(twist)
                else:
                    proceed_ok = True
                    for i, pose in enumerate(self.turtle_poses):
                        obstruction_angle = self.calc_obstruction_angle(pose)
                        if (
                            abs(obstruction_angle) < math.pi / 2.0
                            and distance_to_turtles[i] <= self.distance_min_to_turtles
                        ):
                            self.mitigate_obstruction(obstruction_angle)
                            proceed_ok = False
                            break

                    if proceed_ok:
                        self.move_to_point(twist)
                    else:
                        self.stop_motion(twist)
        else:
            self.stop_motion(twist)
        self.cmd_vel_publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    follower.spawn_follower()
    rclpy.spin(follower)
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
