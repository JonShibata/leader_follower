// Copyright 2025 Jon Shibata
// Licensed under the MIT License

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include "../src/turtle2.hpp"


TEST (LeaderTest, LeaderPoseCallbackTest) {
  // Create an instance of the Turtle2 class
  Turtle2 turtle2;

  // Create a test message
  auto msg = std::make_shared<turtlesim::msg::Pose>();
  msg->x = 1.0;
  msg->y = 2.0;
  msg->theta = 3.0;

  // Call the leaderPoseCallback function
  turtle2.testLeaderPoseCallback(msg);

  // Check if the leader_pose_ variable is updated correctly
  turtlesim::msg::Pose leader_pose_ = turtle2.leader_pose();

  EXPECT_EQ(leader_pose_.x, 1.0);
  EXPECT_EQ(leader_pose_.y, 2.0);
  EXPECT_EQ(leader_pose_.theta, 3.0);

  // Check if the data_received_ array is updated correctly
  EXPECT_TRUE(turtle2.test_data_received(0));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
