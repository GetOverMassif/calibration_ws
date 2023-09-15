#include "imu_calibration/IMUCalibrationNode.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto pIMUCalibNode = std::make_shared<IMUCalibrationNode>();
    rclcpp::spin(pIMUCalibNode);
    rclcpp::shutdown();
    return 0;
}