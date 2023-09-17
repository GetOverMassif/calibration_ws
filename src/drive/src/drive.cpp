#include "drive/DriveNode.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto pDriveNode = std::make_shared<DriveNode>();
    rclcpp::spin(pDriveNode);
    rclcpp::shutdown();

    return 0;
}