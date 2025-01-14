#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
import math


class Turtle1(Node):
    def __init__(self):
        super().__init__("turtle1")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.leader_pose_subscriber = self.create_subscription(
            Pose, "/leader/pose", self.leader_pose_callback, 10
        )
        self.follower_pose_sub_ = self.create_subscription(
            Pose, "follower/pose", self.follower_pose_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle2/pose", self.turtle2_pose_callback, 10
        )

        self.cli = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = TeleportAbsolute.Request()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now()

        self.current_pose = Pose()
        self.leader_pose = None
        self.follower_pose = None

        self.spawn_x = 5.5
        self.spawn_y = 5.5
        self.spawn_theta = 0.0

        self.moving_right = False
        self.target_x = None
        self.target_y = None
        self.x_threshold_for_leader = 99.9
        self.y_threshold_for_leader = 5.5

        self.distance_min_to_others = 1.0
        self.desired_speed = 4.0

    def pose_callback(self, msg):
        self.current_pose = msg

    def leader_pose_callback(self, msg):
        self.leader_pose = msg

    def follower_pose_callback(self, msg):
        self.follower_pose = msg

    def turtle2_pose_callback(self, msg):
        self.turtle2_pose = msg

    def send_request(self):
        self.req.x = self.spawn_x
        self.req.y = self.spawn_y
        self.req.theta = self.spawn_theta
        self.future = self.cli.call_async(self.req)

    def distance_between_points(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2.0 + (point1.y - point2.y) ** 2.0)

    def timer_callback(self):
        if (
            self.current_pose is None
            or self.leader_pose is None
            or self.follower_pose is None
        ):
            return

        # capture the x position of the leader turtle when it crosses y threshold
        if self.leader_pose.y > self.y_threshold_for_leader and self.target_x is None:
            self.target_x = self.leader_pose.x

        if self.leader_pose.x > self.x_threshold_for_leader and self.target_y is None:
            self.target_y = self.leader_pose.y

        if (self.target_x is not None and self.current_pose.x < self.target_x) or (
            self.target_y is not None and self.current_pose.y < self.target_y
        ):
            # proceed to the lead turtle's path when it crosses y threshold

            distance_to_leader = self.distance_between_points(
                self.current_pose, self.leader_pose
            )
            distance_to_follower = self.distance_between_points(
                self.current_pose, self.follower_pose
            )
            distance_to_turtle2 = self.distance_between_points(
                self.current_pose, self.turtle2_pose
            )
            if (
                distance_to_leader < self.distance_min_to_others
                or distance_to_follower < self.distance_min_to_others
                or distance_to_turtle2 < self.distance_min_to_others
                or self.current_pose.x > 10
                or self.current_pose.y > 10
                or self.current_pose.x < 1.0
                or self.current_pose.y < 1.0
            ):
                self.stop_motion()
            else:
                twist = Twist()
                twist.linear.x = self.desired_speed
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
        else:
            self.stop_motion()

    def stop_motion(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    turtle1 = Turtle1()
    turtle1.send_request()
    rclpy.spin(turtle1)
    turtle1.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
