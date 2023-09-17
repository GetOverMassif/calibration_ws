from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="camera_calibration",
            executable="camera_calibration",
            name="camera_calibration",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"inputSettingsFile": "/root/agv_calibration/src/camera_calibration/camera.xml"},
                {"winSize": 5},
                {"camTopic": "camera"}
            ]
        )
    ])