from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="drive",
            executable="drive",
            name="drive",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"max_vel": 1.0},
                {"v_T": 12.56},
                {"phi": 0.0},
                {"T": 0.01},
                {"cmd_vel_topic": "/demo/cmd_demo"}
            ]
        )
    ])