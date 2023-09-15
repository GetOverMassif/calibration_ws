#include <iostream>
#include <chrono>

#include <cstdio>
#include <functional>

#include "rclcpp/rclcpp.hpp"
// #include "ament_index_cpp/get_package_share_directory.hpp"

#include "imu_tk/calibration.h"
#include "imu_tk/io_utils.h"
#include "imu_tk/filters.h"
#include "imu_tk/integration.h"

using namespace std::chrono_literals;
// using namespace ament_index_cpp;

using namespace std;

class IMUCalibrationNode : public rclcpp::Node
{
public:
    IMUCalibrationNode() : Node("imu_calibration")
    {
        this->declare_parameter("still_duration", 5.0);  // 静态间隔时间
        this->declare_parameter("win_size", 100);   // 中间值连续采样数量

        // std::string file_name = get_package_share_directory("imu_calibration");
        std::string pkg_path = "/home/lj/Documents/aubo/ros2_ws/src/imu_calibration";

        acc_file_name = pkg_path + "/data/data2/acc_data.txt";
        gyro_file_name = pkg_path + "/data/data2/gyro_data.txt";
        calib_save_path = pkg_path + "/data/data2";
        
        this->declare_parameter("acc_file_name", acc_file_name);   // 中间值连续采样数量
        this->declare_parameter("gyro_file_name", gyro_file_name);   // 中间值连续采样数量
        this->declare_parameter("calib_save_path", calib_save_path);   // 中间值连续采样数量

        double still_duration = this->get_parameter("still_duration").as_double();
        int win_size = this->get_parameter("win_size").as_int();

        acc_file_name = this->get_parameter("acc_file_name").as_string();
        gyro_file_name = this->get_parameter("gyro_file_name").as_string();
        calib_save_path = this->get_parameter("calib_save_path").as_string();

        std::vector<imu_tk::TriadData> acc_data, gyro_data;
        
        std::cout << "Importing acc data from the Matlab matrix file : " << acc_file_name << std::endl;  
        importAsciiData(acc_file_name.c_str(), acc_data, imu_tk::TIMESTAMP_UNIT_SEC);
        std::cout << "Importing gyro data from the Matlab matrix file : " << gyro_file_name << std::endl;  
        importAsciiData(gyro_file_name.c_str(), gyro_data, imu_tk::TIMESTAMP_UNIT_SEC);

        std::cout << "acc_data.size() = " << acc_data.size() << std::endl;
        std::cout << "gyro_data.size() = " << gyro_data.size() << std::endl;

        // // if (argc==3) {
        // //     size_t lastSeparator = acc_file_name.find_last_of("/\\");
        // //     if (lastSeparator != std::string::npos) {
        // //         calib_save_path = acc_file_name.substr(0, lastSeparator) + "/";
        // //         // std::cout << "Parent directory: " << calib_save_path << std::endl;
        // //     } else {
        // //         // std::cout << "No path separator found." << std::endl;
        // //     }
        // // }else if (argc==4) {
        // //     calib_save_path = argv[3];
        // //     if (calib_save_path[-1] != '/') {
        // //     calib_save_path += '/';
        // //     }
        // // }

        // std::cout << "Calibration save path: " << calib_save_path << std::endl;
        
        // 首先给加速度标定参数设置偏移量， 给陀螺仪标定参数设置比例因子
        imu_tk::CalibratedTriad init_acc_calib, init_gyro_calib;
        init_acc_calib.setBias( Eigen::Vector3d(32768, 32768, 32768) );
        init_gyro_calib.setScale( Eigen::Vector3d(1.0/6258.0, 1.0/6258.0, 1.0/6258.0) );

        init_acc_calib.setBias( Eigen::Vector3d(0, 0, 0) );
        init_gyro_calib.setScale( Eigen::Vector3d(1, 1, 1) );

        mp_calib = new imu_tk::MultiPosCalibration();

        mp_calib->setInitStaticIntervalDuration(still_duration);
        // 设置加速度计、陀螺仪的初始猜测校准参数
        mp_calib->setInitAccCalibration( init_acc_calib );
        mp_calib->setInitGyroCalibration( init_gyro_calib );
        // 设置重力加速度
        mp_calib->setGravityMagnitude(9.81744);
        // 激活详细输出
        mp_calib->enableVerboseOutput(true);
        // 如果参数为true，则使用每个静态间隔的平均加速度而不是所有样本来获得加速度计校准。默认为false。
        mp_calib->enableAccUseMeans(false);
        //mp_calib.setGyroDataPeriod(0.01);
        // 进行标定
        mp_calib->calibrateAccGyro(acc_data, gyro_data, win_size);

        mp_calib->getAccCalib().save(calib_save_path + "/test_imu_acc.calib");
        mp_calib->getGyroCalib().save(calib_save_path + "/test_imu_gyro.calib");

        RCLCPP_INFO(this->get_logger(), "Writng AccCalib into : '%s'", (calib_save_path + "/test_imu_acc.calib").c_str());
        RCLCPP_INFO(this->get_logger(), "Writng GyroCalib into : '%s'", (calib_save_path + "/test_imu_gyro.calib").c_str());

        this->~Node();
    }

private:
    imu_tk::MultiPosCalibration* mp_calib;
    std::string acc_file_name;
    std::string gyro_file_name;
    std::string calib_save_path;
};

