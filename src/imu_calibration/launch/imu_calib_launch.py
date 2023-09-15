from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="imu_calibration",
            executable="imu_calibration",
            name="imu_calibration",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"still_duration":5.0},
                {"win_size":100},
                {"acc_file_name":"/home/lj/Documents/aubo/ros2_ws/src/imu_calibration/data/data4/acc_data.txt"},
                {"gyro_file_name":"/home/lj/Documents/aubo/ros2_ws/src/imu_calibration/data/data4/gyro_data.txt"},
                {"calib_save_path":"/home/lj/Documents/aubo/ros2_ws/src/imu_calibration/data/data4"}
            ]
        )
    ])