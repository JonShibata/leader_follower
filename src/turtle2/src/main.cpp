#include "turtle2.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Turtle2>();
    node->spawnSelf();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}