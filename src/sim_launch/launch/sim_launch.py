from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="turtlesim", executable="turtlesim_node", name="turtlesim"),
            Node(
                package="turtle_control",
                executable="turtle1",
                name="turtle1",
                output="screen",
            ),
            Node(
                package="turtle_control",
                executable="leader",
                name="leader",
                output="screen",
            ),
            Node(
                package="turtle_control",
                executable="follower",
                name="follower",
                output="screen",
            ),
            Node(
                package="turtle2",
                executable="turtle2_node",
                name="turtle2",
                output="screen",
            ),
        ]
    )
