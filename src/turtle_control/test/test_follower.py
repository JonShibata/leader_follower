import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import pytest
import multiprocessing
import unittest
from unittest.mock import MagicMock
import math

from turtle_control.follower import Follower


@pytest.fixture(scope="session", autouse=True)
def set_multiprocessing_start_method():
    multiprocessing.set_start_method("forkserver", force=True)


class TestFollower(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=None)
        self.node = Follower()
        self.node.cmd_vel_publisher_ = MagicMock()
        self.node.cmd_vel_publisher_.publish = MagicMock()
        self.node.input_request_publisher_ = MagicMock()
        self.node.input_request_publisher_.publish = MagicMock()

    def tearDown(self):
        rclpy.shutdown()

    def test_stop_motion(self):
        twist = Twist()
        self.node.stop_motion(twist)
        self.assertEqual(twist.linear.x, 0.0)
        self.assertEqual(twist.angular.z, 0.0)

    def test_slow_backup(self):
        twist = Twist()
        self.node.slow_backup(twist)
        self.assertEqual(twist.linear.x, -0.1)
        self.assertEqual(twist.angular.z, 0.0)

    def test_calc_obstruction_angle(self):

        self.node.turtle1_pose = MagicMock()
        self.node.current_pose = MagicMock()

        # when turtle1 is north and theta is zero | then pi/2
        self.node.turtle1_pose.x = 1.0
        self.node.turtle1_pose.y = 1.0
        self.node.current_pose.x = 1.0
        self.node.current_pose.y = 0.0
        self.node.current_pose.theta = 0.0

        obstruction_angle = self.node.calc_obstruction_angle(self.node.turtle1_pose)

        self.assertAlmostEqual(obstruction_angle, math.pi / 2, 2)

        # when turtle1 is north and theta is p2/2 | then 0.0
        self.node.turtle1_pose.x = 1.0
        self.node.turtle1_pose.y = 1.0
        self.node.current_pose.x = 1.0
        self.node.current_pose.y = 0.0
        self.node.current_pose.theta = math.pi / 2.0

        obstruction_angle = self.node.calc_obstruction_angle(self.node.turtle1_pose)

        self.assertAlmostEqual(obstruction_angle, 0.0, 2)

        # when turtle1 is north and theta is p2 | then -pi/2
        self.node.turtle1_pose.x = 1.0
        self.node.turtle1_pose.y = 1.0
        self.node.current_pose.x = 1.0
        self.node.current_pose.y = 0.0
        self.node.current_pose.theta = math.pi

        obstruction_angle = self.node.calc_obstruction_angle(self.node.turtle1_pose)

        self.assertAlmostEqual(obstruction_angle, -math.pi / 2.0, 2)

        # when turtle1 is north and theta is -p2 | then -pi/2
        self.node.turtle1_pose.x = 1.0
        self.node.turtle1_pose.y = 1.0
        self.node.current_pose.x = 1.0
        self.node.current_pose.y = 0.0
        self.node.current_pose.theta = -math.pi

        obstruction_angle = self.node.calc_obstruction_angle(self.node.turtle1_pose)

        self.assertAlmostEqual(obstruction_angle, -math.pi / 2.0, 2)

    def test_mitigate_obstruction(self):

        # when obstruction is within 45 deg of heading | then turn is first choice
        self.node.mitigate_obstruction(math.pi / 4.0 - 0.01)

        self.node.input_request_publisher_.publish.assert_called_once()
        msg = self.node.input_request_publisher_.publish.call_args[0][0]

        self.assertIsInstance(msg, String)
        self.assertEqual(msg.data.split("\n")[0], "turn left")
        self.assertEqual(msg.data.split("\n")[2], "turn right")

        # when obstruction is within 45 deg of heading | then turn is first choice
        self.node.mitigate_obstruction(-math.pi / 4.0 + 0.01)

        msg = self.node.input_request_publisher_.publish.call_args[0][0]

        self.assertIsInstance(msg, String)
        self.assertEqual(msg.data.split("\n")[0], "turn right")
        self.assertEqual(msg.data.split("\n")[2], "turn left")

        # when obstruction is outside 45 deg of heading | then circle is first choice
        self.node.mitigate_obstruction(math.pi / 4.0 + 0.01)

        msg = self.node.input_request_publisher_.publish.call_args[0][0]

        self.assertIsInstance(msg, String)
        self.assertEqual(msg.data.split("\n")[0], "circle around")


if __name__ == "__main__":
    unittest.main()
