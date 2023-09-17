#pragma once

#include <iostream>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int16.hpp"
#include "agv_base_msgs/msg/encoder.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "cam_odo_calibration/data_selection.hpp"
#include "cam_odo_calibration/Solver.hpp"
#include "cam_odo_calibration/Simulator.hpp"
#include "cam_odo_calibration/PointCloudAlign.hpp"

// 时间戳同步问题
// 时间戳的硬件同步可以消除时钟源漂移的影响
// 当时间戳由传感器产生时不存在数据传输延迟，但存在时钟源不一致，需要硬件同步即提供统一的时钟源，或软件上算法估计时间戳延迟
// 当时间戳由接受数据方提供如linux，则不存在时钟源不一致的情况，但存在数据传输时间不同的影响，此时只能使用算法估计，无法硬件同步
// 不用考虑

// 相机里程计如何提供，是标签已知位置还是未知，需不需要自己写
// 会提供局部定位（相机到标签的位姿）和全局定位，如果标签没有全局位置，只能使用局部位置
// 相机坐标系

// 标签定位是提供api还是topic，各传感器数据是通过topic传递？
// topic提供

using namespace std::chrono_literals;
using std::placeholders::_1;

class CalibrationNode : public rclcpp::Node
{
public:
    CalibrationNode()
    : Node("cam_odo_calibration"), ds_(1), solver_(sync_result_, camDatas_), T_(10), r_(0.3), b_(1.25), pca_(1),
    simulator(T_)
    {
        this->declare_parameter("camTopic", "");
        this->declare_parameter("odoTopic", "");
        this->declare_parameter("encoderTopic", "");
        this->declare_parameter("scanTopic", "");
        this->declare_parameter("maxdT", 0.5);
        this->declare_parameter("mindT", 0.1);
        this->declare_parameter("min_dtheta", 0.1);
        this->declare_parameter("min_d", 0.05);
        this->declare_parameter("min_range", 0.1);
        this->declare_parameter("min_num_points", 10);
        this->declare_parameter("min_intensity", 0.1);
        this->declare_parameter("reflector", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        camTopic_ = this->get_parameter("camTopic").as_string();
        odoTopic_ = this->get_parameter("odoTopic").as_string();
        scanTopic_ = this->get_parameter("scanTopic").as_string();
        encoderTopic_ = this->get_parameter("encoderTopic").as_string();
        maxdT_ = this->get_parameter("maxdT").as_double();
        mindT_ = this->get_parameter("mindT").as_double();
        min_dtheta_ = this->get_parameter("min_dtheta").as_double();
        min_d_ = this->get_parameter("min_d").as_double();
        min_range_ = this->get_parameter("min_range").as_double();
        min_intensity_ = this->get_parameter("min_intensity").as_double();
        min_num_points_ = this->get_parameter("min_num_points").as_int();

        // 反光柱位置(x1,y1,x2,y2...)
        std::vector<double> reflector = this->get_parameter("reflector").as_double_array();
        // convert double vector into Eigen::Vector2d
        for (int i = 0; i < reflector.size(); i += 2) {
            Eigen::Vector2d v(reflector[i], reflector[i + 1]);
            reflector_.push_back(v);
        }
        // calculate distance between one point and others
        for (int i = 0; i < reflector_.size(); ++i) {
            double dist = 0.0;
            for (int j = 0; j < reflector_.size(); ++j) {
                if (i != j) {
                    dist += (reflector_[i] - reflector_[j]).norm();
                }
            }
            vdist_reflector_.push_back(dist);
        }

        J(0, 0) = r_ / 2;
        J(0, 1) = r_ / 2;
        J(1, 0) = -r_ / b_;
        J(1, 1) = r_ / b_;

        std::chrono::milliseconds T(T_);
        // timer_ = this->create_wall_timer(T, std::bind(&CalibrationNode::timer_callback, this));
        
        encoder_sub_ = this->create_subscription<agv_base_msgs::msg::Encoder>(
            encoderTopic_, 200, std::bind(&CalibrationNode::encoder_callback, this, _1)
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odoTopic_, 200, std::bind(&CalibrationNode::odom_callback, this, _1)
        );

        camera_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            camTopic_, 200, std::bind(&CalibrationNode::camera_callback, this, _1)
        );

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scanTopic_, 200, std::bind(&CalibrationNode::scan_callback, this, _1)
        );
        // 手动发生标定指令
        calibrate_sub_ = this->create_subscription<std_msgs::msg::Int16>(
            "calibrate", 1, std::bind(&CalibrationNode::cali_callback, this, _1)
        );

    }

    void timer_callback();

    void camera_callback(const geometry_msgs::msg::TransformStamped&);

    void scan_callback(const sensor_msgs::msg::LaserScan&);

    void odom_callback(const nav_msgs::msg::Odometry&);

    void cali_callback(const std_msgs::msg::Int16&);

    void encoder_callback(const agv_base_msgs::msg::Encoder&);

    Eigen::Matrix4d ROSPose2EigenMatrix(const geometry_msgs::msg::Transform&);

    void Scan2vPoints(const sensor_msgs::msg::LaserScan&, std::vector<Eigen::Vector2d>&);
    
    void segment(const std::vector<Eigen::Vector2d>&, std::vector<std::vector<Eigen::Vector2d>>&);

    void getCenters(const std::vector<std::vector<Eigen::Vector2d>>&, std::vector<Eigen::Vector2d>&);

    void match(const std::vector<Eigen::Vector2d>&, std::vector<int>&);

private:
    rclcpp::TimerBase::SharedPtr timer_;

    // reflector_ is the position of the reflector in the global coordinate system
    std::vector<Eigen::Vector2d> reflector_;
    // vector stores distance bewtween one point and others
    std::vector<double> vdist_reflector_;

    std::string camTopic_;
    std::string odoTopic_;
    std::string scanTopic_;
    std::string encoderTopic_;

    double maxdT_;
    double mindT_;
    // min_dtheta_ is the required minimum rotation angle of the camera
    double min_dtheta_;
    // min_d_ is the required minimum displacement of the camera
    double min_d_;
    // min_range_ is the required minimum distance between two points in the same segment
    double min_range_;
    // min_intensity_ is the required minimum intensity of the point
    double min_intensity_;
    // min_num_points_ is the required minimum number of points in one segment
    int min_num_points_;

    int T_;
    std::vector<data_selection::odo_data> odoDatas_;
    std::vector<data_selection::cam_data> camDatas_;
    std::vector<data_selection::cam_data> scanDatas_;
    std::vector<data_selection::sync_data> sync_result_;

    data_selection ds_;
    Solver solver_;

    Simulator simulator;

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr camera_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr calibrate_sub_;
    rclcpp::Subscription<agv_base_msgs::msg::Encoder>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    double r_, b_;
    Eigen::Matrix2d J;
    
    // for scan
    std::vector<Eigen::Vector2d> vPoints_;
    std::vector<std::vector<Eigen::Vector2d>> segments_;
    std::vector<Eigen::Vector2d> centers_;

    PointCloudAlign pca_;  // 点云对齐
};

// 仿真
// 数据个数和结果有关系，这是为什么？ 50个效果是最好的(圆弧轨迹，逆时针)
// 如果相机外参旋转矩阵rpy都很一般，则误差比较大，原因还未知
// 这里试的是无噪声的数据，在没有噪声的情况下，估计的值和真实值仍有误差(why?)
// 数据对结果影响很大，最好轮速变化明显且左转右转都有,S型曲线效果应该很好
void CalibrationNode::timer_callback()
{
    // if (camDatas_.size() < 100)
    //     simulator.generate(camDatas_, odoDatas_); 
    // else {
    //     std::cout << "odoDatas size: " << odoDatas_.size() << std::endl;
    //     std::cout << "camDatas size: " << camDatas_.size() << std::endl;
    //     ds_.selectData(odoDatas_, camDatas_, sync_result_);
    //     std::cout << "sync_result size: " << sync_result_.size() << std::endl;
    //     solver_.calibrate(4, 0);
    //     std::cout << '\n' << colouredString("-------True Params-------", GREEN, BOLD) << '\n' 
    //              << colouredString("Axle between wheels: ", RED, BOLD) << simulator.b << '\n' 
    //              << colouredString("cam-odom x: ", RED, BOLD) << simulator.toc(0) << '\n'
    //              << colouredString("cam-odom y: ", RED, BOLD) << simulator.toc(1) << '\n' 
    //              << colouredString("Left wheel radius: ", RED, BOLD) << simulator.r << '\n'
    //              << colouredString("Right wheel radius: ", RED, BOLD) << simulator.r  << std::endl;
    //     cout << colouredString("true Roc: ", RED, BOLD) << "\n" << simulator.Roc.matrix() << endl;
    //     timer_->cancel();
    // }
    static int first;
    if (!first) {
        simulator.transform();
        first = 1;
        Eigen::Matrix3d T = pca_.align2d(simulator.reflectors, simulator.centers, simulator.matched_gb);
        Eigen::Matrix3d T_true = Eigen::Matrix3d::Identity();
        T_true.block<2, 2>(0, 0) = simulator.R_scan.transpose();
        T_true.block<2, 1>(0, 2) = -simulator.R_scan.transpose() * simulator.t_scan;
        cout << colouredString("estimate T: ", RED, BOLD) << "\n" << T.matrix() << endl;
        cout << colouredString("true T: ", RED, BOLD) << "\n" << T_true.matrix() << endl;
    }
}

void CalibrationNode::camera_callback(const geometry_msgs::msg::TransformStamped& cur_pose) {
    static int start;
    static geometry_msgs::msg::TransformStamped last_pose;

    if (start == 0) {
        start = 1;
    }
    else {
        double cur_time = cur_pose.header.stamp.sec + cur_pose.header.stamp.nanosec * 1e-3;
        double last_time = last_pose.header.stamp.sec + last_pose.header.stamp.nanosec * 1e-3;
        // 主要看发布定位的topic是连续发布还是识别到了再发布
        data_selection::cam_data cam;
        cam.start_t = last_time;
        cam.end_t = cur_time;
        
        // c: 当前相机位姿，l: 上一时刻相机位姿 b: 标定板
        Eigen::Matrix4d Tcb = ROSPose2EigenMatrix(cur_pose.transform);
        Eigen::Matrix4d Tlb = ROSPose2EigenMatrix(last_pose.transform);

        Eigen::Matrix4d Tlc = Tlb * Tcb.inverse();
        Eigen::Matrix3d Rlc = Tlc.block<3, 3>(0, 0);
        Eigen::Vector3d tlc = Tlc.block<3, 1>(0, 3);

        Eigen::AngleAxisd rotation_vector(Rlc);
        Eigen::Vector3d axis = rotation_vector.axis();
        double deltatheta = rotation_vector.angle();

        if (axis(1) > 0) {
            deltatheta *= -1;
            axis *= -1;
        }
        
        if (deltatheta > min_dtheta_ && tlc.norm() > min_d_) {
            cam.axis = axis;
            cam.deltaTheta = deltatheta;
            cam.Rcl = Rlc.transpose();
            cam.tlc = tlc;
            camDatas_.push_back(cam);
        }
        else return;
    }
    last_pose = cur_pose;
}

Eigen::Matrix4d CalibrationNode::ROSPose2EigenMatrix(const geometry_msgs::msg::Transform& pose) {
    geometry_msgs::msg::Quaternion ros_q = pose.rotation;

    Eigen::Quaterniond q(ros_q.w, ros_q.x, ros_q.y, ros_q.z);
    Eigen::Vector3d t(pose.translation.x, pose.translation.y, pose.translation.z);

    Eigen::Matrix3d R = q.toRotationMatrix();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

void CalibrationNode::odom_callback(const nav_msgs::msg::Odometry& odom) {
    data_selection::odo_data odoData;
    odoData.time = odom.header.stamp.sec + 1e-9 * odom.header.stamp.nanosec;
    Eigen::Vector2d v = J.inverse() * Eigen::Vector2d(odom.twist.twist.linear.x, odom.twist.twist.angular.z);
    odoData.v_left = v(0);
    odoData.v_right = v(1);
    odoDatas_.push_back(odoData);
}

void CalibrationNode::cali_callback(const std_msgs::msg::Int16& cali) {
    std::cout << "odoDatas size: " << odoDatas_.size() << std::endl;
    if (cali.data == 0){
        std::cout << "camDatas size: " << camDatas_.size() << std::endl;
        for (auto &x : camDatas_) {
            cout << colouredString("rotation axis: ", BLUE, BOLD) << '\n' << x.axis << endl;
        }
        ds_.sensor_ = 0;
        ds_.selectData(odoDatas_, camDatas_, sync_result_);
        std::cout << "sync_result size: " << sync_result_.size() << std::endl;
        solver_.calibrate(4, 0);
    }
    if (cali.data == 1) {
        std::cout << "scanDatas size: " << scanDatas_.size() << std::endl;
        ds_.sensor_ = 1;
        ds_.selectData(odoDatas_, scanDatas_, sync_result_);
        std::cout << "sync_result size: " << sync_result_.size() << std::endl;
        solver_.calibrate(4, 1);
    }
    std::cout << '\n' << colouredString("-------True Params-------", GREEN, BOLD) << '\n' 
                << colouredString("Axle between wheels: ", RED, BOLD) << 1.25 << '\n' 
                << colouredString("cam-odom x: ", RED, BOLD) << -0.55 << '\n'
                << colouredString("cam-odom y: ", RED, BOLD) << 0.00 << '\n' 
                << colouredString("Left wheel radius: ", RED, BOLD) << 0.30 << '\n'
                << colouredString("Right wheel radius: ", RED, BOLD) << 0.30  << std::endl;
    Eigen::Matrix3d Roc_true;
    Roc_true << 0, -1, 0,
                1, 0, 0,
                0, 0, 1;
    cout << colouredString("true Roc: ", RED, BOLD) << "\n" << Roc_true << endl;
}

void CalibrationNode::encoder_callback(const agv_base_msgs::msg::Encoder& encoder) {
    data_selection::odo_data odoData;
    odoData.time = encoder.header.stamp.sec + 1e-9 * encoder.header.stamp.nanosec;
    odoData.v_left = encoder.left_encoder / 0.05 / 10000.0 / 21.0 * 2.0 * M_PI;
    odoData.v_right = encoder.right_encoder / 0.05 / 10000.0 / 21.0 * 2.0 * M_PI;
    odoDatas_.push_back(odoData);
}

void CalibrationNode::scan_callback(const sensor_msgs::msg::LaserScan& scan) {
    //clear vector
    vPoints_.clear();
    segments_.clear();
    centers_.clear();

    static Eigen::Matrix3d last_pose = Eigen::Matrix3d::Identity();
    static int start;
    static double last_time;
    // convert scan to vPoints_, segment the scan into several segments_, get the center of each segment
    // TODO: 分割、获取中点这部分的逻辑还比较粗糙，有待实用准确度更高的方法
    Scan2vPoints(scan, vPoints_);
    segment(vPoints_, segments_);
    getCenters(segments_, centers_);

    // vmatched_r2c stores the index of matched center of reflector
    std::vector<int> vmatched_r2c(segments_.size(), -1);
    match(centers_, vmatched_r2c);
    
    // error if segments_ size is not equal to reflector size
    // 非反光板的点是否可能构成
    if (segments_.size() != reflector_.size()) {
        std::cout << colouredString("segments_ size is not equal to reflector size", RED, BOLD) << std::endl;
        std::cout << "segments size = " << segments_.size() << std::endl;
        return;
    }

    // get pose of the lidar
    Eigen::Matrix3d cur_pose = pca_.align2d(reflector_, centers_, vmatched_r2c);
    double cur_time = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9;

    if (start == 0) {
        start = 1;
    }
    else {
        data_selection::cam_data scanData;
        scanData.start_t = last_time;
        scanData.end_t = cur_time;

        Eigen::Matrix3d Tlc = last_pose.inverse() * cur_pose;
        Eigen::Matrix3d Rlc = Eigen::Matrix3d::Identity();
        Rlc.block<2, 2>(0, 0) = Tlc.block<2, 2>(0, 0);
        Eigen::Vector2d tlc = Tlc.block<2, 1>(0, 2);

        Eigen::AngleAxisd rotation_vector(Rlc);
        Eigen::Vector3d axis = rotation_vector.axis();
        double deltatheta = rotation_vector.angle();

        if (axis(2) < 0) {
            deltatheta *= -1;
            axis *= -1;
        }
        
        if (fabs(deltatheta) > min_dtheta_ && tlc.norm() > min_d_) {
            std::cout << "scan tlc: " << tlc.norm() << std::endl;
            std::cout << "scan deltatheta: " << deltatheta << std::endl;

            scanData.axis = axis;
            scanData.deltaTheta = deltatheta;
            scanData.Rcl = Rlc.transpose();
            scanData.tlc << tlc(0), tlc(1), 0;
            scanDatas_.push_back(scanData);
        }
        else return;
    }
    last_pose = cur_pose;
    last_time = cur_time;
}

// 扫描转换为散点
void CalibrationNode::Scan2vPoints(const sensor_msgs::msg::LaserScan& scan, std::vector<Eigen::Vector2d>& vPoints) {
    double angle_min = scan.angle_min;
    double angle_increment = scan.angle_increment;
    double range_min = scan.range_min;
    double range_max = scan.range_max;
    std::vector<float> ranges = scan.ranges;
    int size = ranges.size();
    // convert scan to vPoints
    for (int i = 0; i < size; ++i) {
        double angle = angle_min + i * angle_increment;
        double range = ranges[i];
        // judge if range is valid and intensity is large enough
        // intensity is used to filter out the points that don't belong to the board
        // then convert scan to vPoints
        if (range > range_min && range < range_max && scan.intensities[i] > min_intensity_) {
            Eigen::Vector2d vPoint;
            vPoint(0) = range * cos(angle);
            vPoint(1) = range * sin(angle);
            vPoints.push_back(vPoint);
        }
    }
}

// segment the scan into several segments_
void CalibrationNode::segment(const std::vector<Eigen::Vector2d>& vPoints, std::vector<std::vector<Eigen::Vector2d>>& segments) {
    int size = vPoints.size();
    if (size == 0) return;
    std::vector<Eigen::Vector2d> segment;
    segment.push_back(vPoints[0]);
    // two points are in the same segment if the distance between them is less than min_range_
    for (int i = 1; i < size; ++i) {
        Eigen::Vector2d v = vPoints[i] - vPoints[i - 1];
        if (v.norm() < min_range_) {
            segment.push_back(vPoints[i]);
        }
        else {
            if (segment.size() > min_num_points_)
                segments.push_back(segment);
            segment.clear();
            segment.push_back(vPoints[i]);
        }
    }
    if (segment.size() > min_num_points_)
        segments.push_back(segment);
}

void CalibrationNode::getCenters(const std::vector<std::vector<Eigen::Vector2d>>& segments, 
                                std::vector<Eigen::Vector2d>& centers) {
    for (auto &segment : segments) {
        Eigen::Vector2d center;
        int size = segment.size();
        for (int i = 0; i < size; ++i) {
            center += segment[i];
        }
        center /= size;
        centers.push_back(center);
    }
}

// match centers and reflector according to distance between one point and others
// 根据一个点与其他点之间的距离将中心点与反光板位置匹配上
void CalibrationNode::match(const std::vector<Eigen::Vector2d>& centers, std::vector<int>& vmatched_r2c) {
    int size = centers.size();
    // vector stores distance bewtween one point and others
    std::vector<double> vdist_centers(size, 0.0);
    // calculate distance between one point and others
    for (int i = 0; i < size; ++i) {
        double dist = 0.0;
        for (int j = 0; j < size; ++j) {
            if (i != j) {
                dist += (centers[i] - centers[j]).norm();
            }
        }
        vdist_centers[i] = dist;
    }

    // match centers and reflector according to distance between one point and others
    // vmatched_r2c[idx_reflector] = idx_center
    for (int i = 0; i < size; ++i) {
        double min_dist = 1e10;  // min_dist will be min({fabs(vdist_centers[i] - vdist_reflector_[j])}) finally
        int min_index = -1;
        for (int j = 0; j < size; ++j) {
            if (vmatched_r2c[j] == -1) {
                double dist = fabs(vdist_centers[i] - vdist_reflector_[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    min_index = j;
                }
            }
        }
        vmatched_r2c[min_index] = i;
    }
}

