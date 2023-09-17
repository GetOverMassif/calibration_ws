#pragma once

#include "Eigen/Dense"
#include "imu_odo_calibration/data_selection.hpp"
#include <random>

using namespace std;

class Simulator {
public:
    Simulator(double Tms);
    void generate(std::vector<data_selection::cam_data>& cam_datas, std::vector<data_selection::odo_data>& odo_datas);

    void transform();

    int times = 0;

    Eigen::Matrix3d Roc;
    Eigen::Vector3d toc;
    Eigen::Matrix4d Toc;
    
    double v_left;
    double v_right;
    double v;
    double v_last;
    double omega;
    double omega_last;

    double b;
    double r;
    double J11, J12, J21, J22;

    double Ts;

    Eigen::Matrix4d Todo_c;
    Eigen::Matrix4d Tcam_l;
    Eigen::Matrix4d Tcam_c;

    double t_l;
    double t_c;

    std::default_random_engine generator;
    std::normal_distribution<double> dist;

    std::vector<Eigen::Vector2d> centers;
    std::vector<Eigen::Vector2d> reflectors;
    std::vector<int> matched_gb;
    Eigen::Matrix2d R_scan;
    Eigen::Vector2d t_scan;
};

Simulator::Simulator(double Tms): dist(0, 0.01) {
    Eigen::MatrixXd Ryx;
    // Ryx = Eigen::AngleAxisd(15.0 * M_PI / 180.0, Eigen::Vector3d(0, 1, 0)).toRotationMatrix() * 
    //         Eigen::AngleAxisd(15.0 * M_PI / 180.0, Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
    // Roc = Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d(1, 1, 1)).toRotationMatrix();
    Roc << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0;
    toc = {0.36, 0.06, 0.10};
    Toc = Eigen::Matrix4d::Identity();
    Toc.block<3, 3>(0, 0) = Roc;
    Toc.block<3, 1>(0, 3) = toc;

    b = 0.57;
    r = 0.1;
    J11 = r / 2;
    J12 = r / 2;
    J21 = -r / b;
    J22 = r / b;

    

    Ts = Tms / 1000.0;
    
    // create 3 random reflectors point
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-10.0, 10.0);
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector2d reflector;
        reflector(0) = distribution(generator);
        reflector(1) = distribution(generator);
        reflectors.push_back(reflector);
        matched_gb.push_back(i);
    }

    double theta = 15.0 / 180.0 * M_PI;
    R_scan = Eigen::Matrix2d::Identity();
    R_scan << cos(theta), -sin(theta),
            sin(theta), cos(theta);
    t_scan << 5.1, -3.6;
}

void Simulator::generate(std::vector<data_selection::cam_data>& cam_datas, std::vector<data_selection::odo_data>& odo_datas)
{
    t_c = times * Ts;
    v_left = 1.0 * cos(t_c);
    v_right = 1.0 * sin(t_c);
    v = J11 * v_left + J12 * v_right;
    omega = J21 * v_left + J22 * v_right;
    if (times == 0) {
        data_selection::odo_data odo_data;
        odo_data.time = 0.0;
        odo_data.v_left = v_left;
        odo_data.v_right = v_right;
        odo_datas.push_back(odo_data);

        t_l = 0.0;

        Todo_c = Eigen::Matrix4d::Identity();
        Tcam_c = Todo_c * Toc;
        Tcam_l = Tcam_c;       
        v_last = v;
        omega_last = omega; 
    }
    else {
        Eigen::Matrix4d deltaT = Eigen::Matrix4d::Identity();

        double o_theta = omega * Ts;
        double t1, t2;
        if (fabs(o_theta) > 1e-12)
        {
            t1 = sin(o_theta) / o_theta;
            t2 = (1 - cos(o_theta)) / o_theta;
        }
        else
        {
            t1 = 1;
            t2 = 0;
        }

        deltaT.block<2, 2>(0, 0) << cos(o_theta), -sin(o_theta),
                                    sin(o_theta), cos(o_theta);
        deltaT.block<2, 1>(0, 3) << v * Ts * t1,  v * Ts * t2;
        Todo_c = Todo_c * deltaT;
        Tcam_c = Todo_c * Toc;

        if (times % 1 == 0) {
            data_selection::odo_data odo_data;
            odo_data.time = t_c;
            odo_data.v_left = v_left + dist(generator);
            odo_data.v_right = v_right + dist(generator);
            odo_datas.push_back(odo_data);
        }

        if (times % 5 == 0) {
            data_selection::cam_data cam_data;
            cam_data.start_t = t_l;
            cam_data.end_t = t_c;
            Eigen::Matrix4d Tcl;
            Tcl = Tcam_c.inverse() * Tcam_l;
            Eigen::Matrix3d Rcl = Tcl.block<3, 3>(0, 0);
            Eigen::Vector3d tlc = Tcl.inverse().block<3, 1>(0, 3);
            Eigen::AngleAxisd rotation_vector(Rcl.transpose());
            Eigen::Vector3d axis = rotation_vector.axis();
            double deltaThtea = rotation_vector.angle();
            if (axis(2) < 0) {
                deltaThtea *= -1;
                axis *= -1;
            }
            cam_data.deltaTheta = deltaThtea;
            cam_data.axis = axis;
            cam_data.Rcl = Rcl;
            cam_data.tlc = tlc;
            cam_datas.push_back(cam_data);

            t_l = t_c;
            Tcam_l = Tcam_c;
        }
    }
    v_last = v;
    omega_last = omega;
    times++;
}

void Simulator::transform() {
    // transform the reflectors
    centers.clear();
    for (int i = 0; i < reflectors.size(); ++i) {
        Eigen::Vector2d center = R_scan * reflectors[i] + t_scan;
        // add noise
        center(0) += dist(generator);
        center(1) += dist(generator);
        centers.push_back(center);
    }
}
