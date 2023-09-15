# pragma once

#include <iostream>
#include <functional>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"


using namespace std::chrono_literals;

// using std::placeholders::_1;

class IMUExtrCalibrationNode : public rclcpp::Node
{
public:
    IMUExtrCalibrationNode()
    : Node("imu_extr_calibration")
    {
        this->declare_parameter("imuTopic", "");
        
        
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    std::string imuTopic;
    std::string odoTopic;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};