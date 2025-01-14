// create class Turtle2 to be used in the main program

#ifndef TURTLE2_HPP
#define TURTLE2_HPP


#include <algorithm>
#include <iterator>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>

class Turtle2 : public rclcpp::Node {
  public:
  Turtle2();

  void testLeaderPoseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    leaderPoseCallback(msg);
  }

  void testFollowerPoseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
    followerPoseCallback(msg);
  }

  void testUpdateTurtle1Pose(const turtlesim::msg::Pose::SharedPtr msg) {
    updateTurtle1Pose(msg);
  }

  void testUpdateTurtle2Pose(const turtlesim::msg::Pose::SharedPtr msg) {
    updateTurtle2Pose(msg);
  }

  double testDistanceBetweenTurtles(turtlesim::msg::Pose pose1_, turtlesim::msg::Pose pose2_) {
    return distanceBetweenTurtles(pose1_, pose2_);
  }

  void testMoveTurtle2(void) {
    moveTurtle2();
  }

  bool test_data_received(int i) {
    return data_received_[i];
  }

  turtlesim::msg::Pose leader_pose() {
    return leader_pose_;
  }

  void spawnSelf();

  private:
  void leaderPoseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  void followerPoseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  void updateTurtle1Pose(const turtlesim::msg::Pose::SharedPtr msg);
  void updateTurtle2Pose(const turtlesim::msg::Pose::SharedPtr msg);
  double distanceBetweenTurtles(turtlesim::msg::Pose pose1_, turtlesim::msg::Pose pose2_);
  void moveTurtle2();

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr leader_pose_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr follower_pose_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool data_received_[4] = {false, false, false, false};
  turtlesim::msg::Pose leader_pose_;
  turtlesim::msg::Pose follower_pose_;
  turtlesim::msg::Pose turtle1_pose_;
  turtlesim::msg::Pose turtle2_pose_;
};


#endif  // TURTLE2_HPP
