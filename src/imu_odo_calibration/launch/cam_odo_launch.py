from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="imu_odo_calibration",
            executable="calibration",
            name="imu_odo_calibration",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"imuTopic": "/imu"},
                {"odoTopic": "/demo/odom_demo"},
                {"scanTopic": "/scan"},
                {"encoderTopic": "/encoder"},
                {"maxdT": 0.5},
                {"mindT": 0.01},
                {"min_dtheta": 0.08},
                {"min_d": 0.05},
                {"min_range": 0.1},
                {"min_num_points": 5},
                {"min_intensity": -0.1},
                {"reflector": [-1.326, -3.805, 6.842, -2.4555, -0.286, 5.295, 5.6575, 0.846]},
            ]
        )
    ])