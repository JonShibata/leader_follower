#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class InputService(Node):
    def __init__(self):
        super().__init__("input_service")
        self.input_response_pub_ = self.create_publisher(String, "input_response", 10)

        self.input_request_sub_ = self.create_subscription(
            String, "input_request", self.get_input_callback, 10
        )

    def get_input_callback(self, msg):

        options = msg.data.split("\n")[0:-1]
        self.get_logger().info(
            ">>> Obstruction detected. Select an option to avoid the obstruction:"
        )
        count = len(options) - 1
        for option in options:
            print(f"{count:>3} - {option}")
            count -= 1

        try:
            index = len(options) - 1 - int(input("\n--- select option: "))
        except Exception as e:
            print(e)
            print("Invalid selection: NOT_A_NUMBER\n ---> Please retry")
            self.get_input_callback(msg)

        if index < 0 or len(options) <= index:
            print("Invalid selection: NOT_IN_LIST\n ---> Please retry")
            self.get_input_callback(msg)
        else:
            self.get_logger().info(f"{options[index]} selected\n")

        response = String()
        response.data = options[index]
        self.input_response_pub_.publish(response)


def main(args=None):
    rclpy.init(args=args)
    node = InputService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
