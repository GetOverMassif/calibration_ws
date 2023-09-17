#include "cam_odo_calibration/CalibrationNode.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto pCalibrationNode = std::make_shared<CalibrationNode>();
  rclcpp::spin(pCalibrationNode);
  rclcpp::shutdown();

  return 0;
}
