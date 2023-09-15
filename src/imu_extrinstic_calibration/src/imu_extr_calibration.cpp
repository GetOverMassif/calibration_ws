#include "imu_extrinstic_calibration/IMUExtrCalibrationNode.hpp"


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto pIMUExtrCalibNode = std::make_shared<IMUExtrCalibrationNode>();
    rclcpp::spin(pIMUExtrCalibNode);
    rclcpp::shutdown();

    return 0;
}