from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    controller_node = Node(
        package="jetracer_py",
        node_executable="controller",
        output="screen",
        emulate_tty=True
    )
    teleop_node = Node(
        package="jetracer_py",
        node_executable="teleop",
        output="screen",
        emulate_tty=True
    )

    return LaunchDescription([
        controller_node,
        teleop_node
    ])
