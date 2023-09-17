# pragma once

#include <iostream>
#include <functional>
#include <chrono>
#include <string>


#include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"


using namespace std::chrono_literals;
using std::placeholders::_1;

class IMUExtrCalibrationNode : public rclcpp::Node
{
public:
    IMUExtrCalibrationNode()
    : Node("imu_extr_calibration")
    {
        this->declare_parameter("imuTopic", "");
        this->declare_parameter("odoTopic", "");

        imuTopic = this->get_parameter("imuTopic").as_string();
        odoTopic = this->get_parameter("odoTopic").as_string();
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, 200, std::bind(&IMUExtrCalibrationNode::imu_callback, this, _1)
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            imuTopic, 200, std::bind(&IMUExtrCalibrationNode::imu_callback, this, _1)
        );

    }

    void timer_callback();

    void imu_callback(const sensor_msgs::msg::Imu&);

private:
    rclcpp::TimerBase::SharedPtr timer_;

    std::string imuTopic;
    std::string odoTopic;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

void IMUExtrCalibrationNode::timer_callback()
{

}

void IMUExtrCalibrationNode::initFirstIMUPose(){

}


void IMUExtrCalibrationNode::imu_callback(const sensor_msgs::msg::Imu& imu_msg)
{
    // static Eigen::Matrix3d last_pose = Eigen::Matrix3d::Identity();
    // static int start;
    // static double last_time;

    double t = imu_msg.header.stamp.toSec();
    double dx = imu_msg.linear_acceleration.x;
    double dy = imu_msg.linear_acceleration.y;
    double dz = imu_msg.linear_acceleration.z;
    double rx = imu_msg.angular_velocity.x;
    double ry = imu_msg.angular_velocity.y;
    double rz = imu_msg.angular_velocity.z;
    Eigen::Vector3d acc(dx, dy, dz);
    Eigen::Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;

}