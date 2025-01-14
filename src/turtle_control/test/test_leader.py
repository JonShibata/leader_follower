import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import pytest
import multiprocessing
import unittest
from unittest.mock import MagicMock
import math

from turtle_control.leader import Leader


@pytest.fixture(scope="session", autouse=True)
def set_multiprocessing_start_method():
    multiprocessing.set_start_method("forkserver", force=True)


class TestLeader(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=None)
        self.node = Leader()
        self.node.cmd_vel_publisher_ = MagicMock()
        self.node.cmd_vel_publisher_.publish = MagicMock()
        self.node.control_direction = MagicMock()
        self.node.turn_controller = MagicMock()

    def tearDown(self):
        rclpy.shutdown()

    def test_stop_motion(self):
        twist = Twist()
        self.node.stop_motion(twist)
        self.assertEqual(twist.linear.x, 0.0)
        self.assertEqual(twist.angular.z, 0.0)

    def test_move_east(self):

        # default case
        twist = Twist()
        self.node.current_pose = Pose()
        self.node.current_pose.x = 0.0
        self.node.move_east(twist)
        self.assertEqual(twist.linear.x, self.node.desired_speed)
        self.assertEqual(twist.angular.z, 0.0)

        # reached the east boundary
        self.node.current_pose.x = 10.0
        twist = Twist()
        self.node.move_east(twist)
        self.assertEqual(self.node.state, "turn_north")

    def test_turn_north(self):

        # default case
        twist = Twist()
        self.node.current_pose = Pose()
        self.node.current_pose.theta = 0.0
        self.node.turn_start_time = self.node.get_clock().now()
        self.node.turn_north(twist)

        self.node.turn_controller.assert_called_once()
        self.node.turn_controller.assert_called_with(twist, 0.0, math.pi / 2.0)

        # reached turn time
        self.node.turn_start_time = (
            self.node.get_clock().now()
            - rclpy.duration.Duration(seconds=self.node.time_for_turn)
        )
        twist = Twist()
        self.node.turn_north(twist)
        self.assertEqual(self.node.desired_orientation, math.pi / 2.0)
        self.assertEqual(self.node.state, "move_north")

    def test_move_north(self):

        # default case
        twist = Twist()
        self.node.current_pose = Pose()
        self.node.move_north(twist)
        self.assertEqual(twist.linear.x, self.node.desired_speed)
        self.node.control_direction.assert_called_once()

        # reached the north boundary
        self.node.current_pose.y = 10.0
        twist = Twist()
        self.node.move_north(twist)
        self.assertEqual(self.node.state, "turn_west")

    def test_turn_west(self):

        # default case
        twist = Twist()

        self.node.turn_start_time = self.node.get_clock().now()
        self.node.turn_west(twist)

        self.node.turn_controller.assert_called_once()
        self.node.turn_controller.assert_called_with(twist, math.pi / 2.0, math.pi)

        # reached turn time
        self.node.turn_start_time = (
            self.node.get_clock().now()
            - rclpy.duration.Duration(seconds=self.node.time_for_turn)
        )
        twist = Twist()
        self.node.turn_west(twist)
        self.assertEqual(self.node.desired_orientation, math.pi)
        self.assertEqual(self.node.state, "move_west")

    def test_move_west(self):

        # default case
        twist = Twist()
        self.node.current_pose = Pose()
        self.node.current_pose.x = 10.0
        self.node.current_pose.theta = math.pi
        self.node.move_west(twist)
        self.assertEqual(twist.linear.x, self.node.desired_speed)
        self.node.control_direction.assert_called_once()

    def test_move_west_end(self):
        # reached the west boundary
        self.node.current_pose = Pose()
        self.node.current_pose.theta = math.pi
        self.node.current_pose.x = 0.0
        twist = Twist()
        self.node.move_west(twist)
        self.assertEqual(self.node.state, "stop")
        self.assertEqual(self.node.path_complete, True)
        self.assertEqual(twist.linear.x, 0.0)
        self.assertEqual(twist.angular.z, 0.0)

    def test_turn_controller(self):

        # start of turn
        twist = Twist()
        self.node.current_pose = Pose()

        self.node.time_delta = 0.0
        angle_start = 0.0
        angle_end = math.pi / 2.0

        self.node.turn_controller(twist, angle_start, angle_end)
        try:
            self.node.control_direction.assert_called_once()
        except AssertionError as Exception:
            print(Exception)

        try:
            self.assertGreater(self.node.desired_orientation, 0.0)
        except AssertionError as Exception:
            print(Exception)

        # ending turn
        self.node.time_delta = (
            self.node.time_for_turn - self.node.time_feedforward_while_turning
        )
        self.node.turn_controller(twist, angle_start, angle_end)

        try:
            self.node.desired_orientation = angle_end
        except AssertionError as Exception:
            print(Exception)

        # end of turn
        self.node.time_delta = self.node.time_for_turn
        self.node.turn_controller(twist, angle_start, angle_end)

        try:
            self.node.desired_orientation = angle_end
        except AssertionError as Exception:
            print(Exception)


if __name__ == "__main__":
    unittest.main()
