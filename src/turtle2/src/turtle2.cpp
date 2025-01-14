// Copyright 2025 Jon Shibata
// Licensed under the MIT License

#include <algorithm>
#include <iterator>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include "turtle2.hpp"


Turtle2::Turtle2() : Node("turtle2") {

  leader_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
          "/leader/pose", 10, std::bind(&Turtle2::leaderPoseCallback, this, std::placeholders::_1));

  follower_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
          "/follower/pose",
          10,
          std::bind(&Turtle2::followerPoseCallback, this, std::placeholders::_1));

  turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
          "/turtle1/pose", 10, std::bind(&Turtle2::updateTurtle1Pose, this, std::placeholders::_1));

  turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
          "/turtle2/pose", 10, std::bind(&Turtle2::updateTurtle2Pose, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

  timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100), std::bind(&Turtle2::moveTurtle2, this));
};


void Turtle2::spawnSelf() {
  auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
  auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
  request->x = 1.0;
  request->y = 4.0;
  request->theta = 3.14 / 2.0;
  request->name = "turtle2";

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(), "Waiting for spawn service...");
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
      == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Turtle2 spawned successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to spawn turtle2");
  }
}


void Turtle2::leaderPoseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
  leader_pose_ = *msg;
  data_received_[0] = true;
}

void Turtle2::followerPoseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
  follower_pose_ = *msg;
  data_received_[1] = true;
}

void Turtle2::updateTurtle1Pose(const turtlesim::msg::Pose::SharedPtr msg) {
  turtle1_pose_ = *msg;
  data_received_[2] = true;
}

void Turtle2::updateTurtle2Pose(const turtlesim::msg::Pose::SharedPtr msg) {
  turtle2_pose_ = *msg;
  data_received_[3] = true;
}

double Turtle2::distanceBetweenTurtles(turtlesim::msg::Pose pose1_, turtlesim::msg::Pose pose2_) {
  return (sqrt(pow(pose1_.x - pose2_.x, 2) + pow(pose1_.y - pose2_.y, 2)));
}

void Turtle2::moveTurtle2() {
  auto twist = geometry_msgs::msg::Twist();

  // Move the turtle in the y direction until it is within 1 unit from the top edge

  if ((turtle2_pose_.y < 11.0 - 1.0)
      && std::all_of(std::begin(data_received_), std::end(data_received_), [](bool i) { return i; })
      && (distanceBetweenTurtles(turtle2_pose_, leader_pose_) > 1.0)
      && (distanceBetweenTurtles(turtle2_pose_, follower_pose_) > 1.0)
      && (distanceBetweenTurtles(turtle2_pose_, turtle1_pose_) > 1.0)) {
    twist.linear.x = 1.5;
    twist.linear.y = 0.0;
    twist.angular.z = 0.0;
  } else {
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.angular.z = 0.0;
  }
  cmd_vel_pub_->publish(twist);
}


// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<Turtle2>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
