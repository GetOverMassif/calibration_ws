#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DriveNode : public rclcpp::Node
{
public:
    DriveNode()
    : Node("drive"), r(0.3), b(1.25)
    {
        this->declare_parameter("max_vel", 0.0);
        this->declare_parameter("v_T", 2.0 * M_PI);
        this->declare_parameter("phi", 0.0);
        this->declare_parameter("T", 0.0);
        this->declare_parameter("cmd_vel_topic", "cmd_vel");

        cmd_vel_ = this->get_parameter("cmd_vel_topic").as_string();
        max_vel_ = this->get_parameter("max_vel").as_double();
        v_T_ = this->get_parameter("v_T").as_double();
        phi_ = this->get_parameter("phi").as_double();
        T_ = this->get_parameter("T").as_double();

        J11 = r / 2;
        J12 = r / 2;
        J21 = -r / b;
        J22 = r / b;

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_, 10);        
        
        std::chrono::milliseconds T(int(T_ * 1000));
        timer_ = this->create_wall_timer(
            T, std::bind(&DriveNode::timer_callback, this)
        );
    }

private:
    void timer_callback();

    std::string cmd_vel_; 

    double max_vel_;
    // 轮速变化周期
    double v_T_;
    double phi_;

    double T_;

    double r, b;
    double J11, J12, J21, J22;

    int n = 0;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

};

void DriveNode::timer_callback()
{
    double t = T_ * n;
    double max_vel = max_vel_;
    if (t > v_T_) {
        max_vel = 0.5 * max_vel_;
    }
    else {
        max_vel = max_vel_;
    }
    
    double v_left = max_vel * sin(2 * M_PI * t / v_T_ + phi_);
    double v_right = max_vel * cos(2 * M_PI * t / v_T_ + phi_);

    double linear = J11 * v_left + J12 * v_right;
    double omega = J21 * v_left + J22 * v_right;

    if (t > 2 * v_T_) {
        linear = 0;
        omega = 0;
    }

    auto vel_msg = geometry_msgs::msg::Twist();
    vel_msg.linear.x = linear; vel_msg.linear.y = 0; vel_msg.linear.z = 0;
    vel_msg.angular.x = 0; vel_msg.angular.y = 0; vel_msg.angular.z = omega;

    vel_publisher_->publish(vel_msg);
    n++;
}
