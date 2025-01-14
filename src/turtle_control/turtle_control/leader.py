#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import math


class Leader(Node):
    def __init__(self):
        super().__init__("leader")

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/leader/cmd_vel", 10)
        self.path_complete_publisher_ = self.create_publisher(
            Bool, "/leader/path_complete", 10
        )
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/leader/pose", self.pose_callback, 10
        )
        self.pose_sub_ = self.create_subscription(
            Pose, "follower/pose", self.follower_pose_callback, 10
        )
        self.turtle1_pose_sub_ = self.create_subscription(
            Pose, "turtle1/pose", self.turtle1_pose_callback, 10
        )
        self.turtle2_pose_sub_ = self.create_subscription(
            Pose, "turtle2/pose", self.turtle2_pose_callback, 10
        )
        self.timer = self.create_timer(0.1, self.move_self)

        self.current_pose = None
        self.follower_pose = None
        self.turtle_poses = [None, None]

        self.state_functions = {
            "move_east": self.move_east,
            "turn_north": self.turn_north,
            "move_north": self.move_north,
            "turn_west": self.turn_west,
            "move_west": self.move_west,
            "stop": self.stop_motion,
        }

        self.state = "move_east"

        self.turtle_width = 1.0  # Approximate width of the turtle
        self.turn_start_time = None
        self.time_delta = 0.0

        self.spawn_x = 2.0
        self.spawn_y = 1.0
        self.desired_orientation = 0.0

        self.distance_min_to_others = 1.0

        self.error_integral_orientation = 0.0
        self.kp_orientation = 7.0
        self.ki_orientation = 3.5
        self.kff_orientation = 4.0

        self.desired_speed = 4.0
        self.desired_speed_turning = 1.5

        self.time_for_turn = 1.0
        self.time_feedforward_while_turning = 0.1

        self.path_complete = False

    def pose_callback(self, msg):
        self.current_pose = msg

    def follower_pose_callback(self, msg):
        self.follower_pose = msg

    def turtle1_pose_callback(self, msg):
        self.turtle_poses[0] = msg

    def turtle2_pose_callback(self, msg):
        self.turtle_poses[1] = msg

    def topic_data_is_valid(self):
        return self.current_pose and self.follower_pose and all(self.turtle_poses)

    def spawn_leader(self):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting to spawn leader")
        request = Spawn.Request()
        request.x = self.spawn_x
        request.y = self.spawn_y
        request.theta = self.desired_orientation
        request.name = "leader"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Spawned leader")
        else:
            self.get_logger().error("Failed to spawn leader")

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

    def distance_to_others_too_close(self):
        for pose in self.turtle_poses:
            distance_to_turtle = self.distance_between_points(self.current_pose, pose)
            if distance_to_turtle < self.distance_min_to_others:
                return True
        return False

    def timer_complete(self):
        time_now = self.get_clock().now()
        time_delta_duration = time_now - self.turn_start_time
        self.time_delta = time_delta_duration.nanoseconds / 1e9
        if self.time_delta < self.time_for_turn:
            return False
        return True

    def control_direction(self):

        error = self.wrap_to_pi(self.desired_orientation - self.current_pose.theta)

        desired_orientation_feedforward = self.kff_orientation * error

        self.error_integral_orientation += error

        return (
            (self.kp_orientation * error)
            + (self.ki_orientation * self.error_integral_orientation)
            + desired_orientation_feedforward
        )

    def turn_controller(self, twist, angle_start, angle_end):

        turn_pct_for_feedforward = min(
            1.0,
            (self.time_delta + self.time_feedforward_while_turning)
            / self.time_for_turn,
        )
        self.desired_orientation = self.wrap_to_pi(
            (angle_end - angle_start) * turn_pct_for_feedforward + angle_start
        )
        twist.linear.x = self.desired_speed_turning
        twist.angular.z = self.control_direction()

    def move_east(self, twist):
        twist.linear.x = self.desired_speed
        if self.current_pose.x >= 10.0 - self.turtle_width:
            self.turn_start_time = self.get_clock().now()
            self.state = "turn_north"

    def turn_north(self, twist):
        angle_start = 0.0
        angle_end = math.pi / 2.0

        if not self.timer_complete():
            self.turn_controller(twist, angle_start, angle_end)
        else:
            self.desired_orientation = angle_end
            self.state = "move_north"

    def move_north(self, twist):
        if self.current_pose.y < 10.0 - self.turtle_width:
            twist.linear.x = self.desired_speed
            twist.angular.z = self.control_direction()
        else:
            self.turn_start_time = self.get_clock().now()
            self.state = "turn_west"

    def turn_west(self, twist):
        angle_start = math.pi / 2.0
        angle_end = math.pi

        if not self.timer_complete():
            self.turn_controller(twist, angle_start, angle_end)
        else:
            self.desired_orientation = angle_end
            self.state = "move_west"

    def move_west(self, twist):
        if self.current_pose.x > self.turtle_width:
            twist.linear.x = self.desired_speed
            twist.angular.z = self.control_direction()
        else:
            self.stop_motion(twist)
            self.state = "stop"
            self.path_complete = True

    def stop_motion(self, twist):
        twist.linear.x = 0.0
        twist.angular.z = 0.0

    def move_self(self):

        if self.topic_data_is_valid():

            twist = Twist()
            if self.distance_to_others_too_close():
                self.stop_motion(twist)
            else:
                self.state_functions[self.state](twist)

            self.cmd_vel_publisher_.publish(twist)

            path_complete_msg = Bool()
            path_complete_msg.data = self.path_complete
            self.path_complete_publisher_.publish(path_complete_msg)


def main(args=None):
    rclpy.init(args=args)
    leader = Leader()
    leader.spawn_leader()
    rclpy.spin(leader)
    leader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
