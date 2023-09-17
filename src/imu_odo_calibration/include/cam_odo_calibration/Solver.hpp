#pragma once

#include "ceres/ceres.h"
#include "imu_odo_calibration/data_selection.hpp"
#include "imu_odo_calibration/CamOdomErr.hpp"
#include "imu_odo_calibration/utils.h"

using namespace std;

template<typename T>
Eigen::Matrix<T, 4, 4> QuaternionMultMatLeft(const Eigen::Quaternion<T>& q)
{
    return (Eigen::Matrix<T, 4, 4>() << q.w(), -q.x(), -q.y(), -q.z(),
                                        q.x(), q.w(), -q.z(), q.y(),
                                        q.y(), q.z(), q.w(), -q.x(),
                                        q.z(), -q.y(), q.x(), q.w()).finished();
}

template<typename T>
Eigen::Matrix<T, 4, 4> QuaternionMultMatRight(const Eigen::Quaternion<T>& q)
{
    return (Eigen::Matrix<T, 4, 4>() << q.w(), -q.x(), -q.y(), -q.z(),
                                        q.x(), q.w(), q.z(), -q.y(),
                                        q.y(), -q.z(), q.w(), q.x(),
                                        q.z(), q.y(), -q.x(), q.w()).finished();
}

// 对应论文中oplus和ominus符号
void oplus_d(const double x1[3], const double x2[3], double res[3])
{
    double c = cos(x1[2]);
	double s = sin(x1[2]);
	double x = x1[0]+c*x2[0]-s*x2[1];
	double y = x1[1]+s*x2[0]+c*x2[1];
 	double theta = x1[2]+x2[2];
	res[0] = x;
	res[1] = y;
	res[2] = theta;
}

void ominus_d(const double x[3], double res[3]) {
	double c = cos(x[2]);
	double s = sin(x[2]);
	res[0] = -c*x[0]-s*x[1];
	res[1] =  s*x[0]-c*x[1];
	res[2] = -x[2];
}

void pose_diff_d(const double pose2[3], const double pose1[3], double res[3]) {
	double temp[3];
	ominus_d(pose1, temp);
	oplus_d(temp, pose2, res);
	
	while(res[2] > +M_PI) res[2] -= 2*M_PI;
	while(res[2] < -M_PI) res[2] += 2*M_PI;
}

class Solver{
public:
    struct calib_result{
        double radius_l, radius_r;  // 左右轮半径
        double b;  // 轮距

        double l[3];  // 车体位姿(x,y,yaw)
        Eigen::Matrix4d Toc;
    };

    Solver(std::vector<data_selection::sync_data>& sync_result, std::vector<data_selection::cam_data>& cam_datas):
        sync_result_(sync_result), cam_datas_(cam_datas)
    {
        Ryx_ = Eigen::Matrix3d::Identity();
    }

    // sensor: 0-camera and odom, 1-lidar, 2-imu, 后两种传感器需传入轮速计内参
    void calibrate(int outliers_iteration, int sensor, double* r=nullptr, double b=0.0);

private:
    bool solve(int max_cond_num);
    bool solve(int max_cond_num, double* r, double b);
    Eigen::VectorXd solveConstraintPlane(const Eigen::MatrixXd& M);
    Eigen::VectorXd x_given_lambda(const Eigen::MatrixXd &M, const double &lambda, const Eigen::MatrixXd &W);
    void computeDisagreement(data_selection::sync_data& calib_data);
    void refineEstimate(Eigen::Matrix4d &Toc, double scale,
                        const std::vector<Eigen::Quaterniond > &quats_odo,
                        const std::vector<Eigen::Vector3d> &tvecs_odo,
                        const std::vector<Eigen::Quaterniond> &quats_cam,
                        const std::vector<Eigen::Vector3d> &tvecs_cam);


    bool estimateRyx();
    void correctCamera();
    bool solveConstraintQyx(Eigen::Vector4d v1, Eigen::Vector4d v2, double &lambda1, double &lambda2);

    void refineExParam();
    
    Eigen::Matrix3d Ryx_;
    std::vector<data_selection::sync_data>& sync_result_;
    std::vector<data_selection::cam_data>& cam_datas_;    
    calib_result res_;
};

bool Solver::estimateRyx()
{
    size_t motion_cnt = sync_result_.size();
    Eigen::MatrixXd M(4 * motion_cnt, 4);
    M.setZero();

    for (int i = 0; i < motion_cnt; ++i) {
        const Eigen::Vector3d& axis = sync_result_[i].axis;
        
        Eigen::Matrix4d M_tmp;
        M_tmp << 0, axis[0], axis[1], axis[2]-1,
                                -axis[0], 0, -1-axis[2], axis[1],
                                -axis[1], 1+axis[2], 0, -axis[0],
                                1-axis[2], -axis[1], axis[0], 0;

        M.block<4, 4>(i * 4, 0) = sin(sync_result_[i].angle / 2) * M_tmp;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector4d v1 = svd.matrixV().block<4, 1>(0, 2);
    Eigen::Vector4d v2 = svd.matrixV().block<4, 1>(0, 3);

    double lambda[2];
    if(!solveConstraintQyx(v1, v2, lambda[0], lambda[1]))
    {
        std::cout << "# ERROR: Quadratic equation cannot be solved due to negative determinant." << std::endl;
        return false;
    }

    Eigen::Matrix3d R_yxs[2];
    double error[2];

    for (int i = 0; i < 2; ++i) {
        double t = lambda[i] * lambda[i] * v1.dot(v1) + 2 * lambda[i] * v2.dot(v1) + v2.dot(v2);

        double lambda2 = sqrt(1.0 / t);
        double lambda1 = lambda[i] * lambda2;

        Eigen::Vector4d x = lambda1 * v1 + lambda2 * v2;
        Eigen::Quaterniond q_yx(x[0], x[1], x[2], x[3]);

        R_yxs[i] = q_yx.toRotationMatrix();

        error[i] = (M * q_yx.coeffs()).norm();
    }

    cout << "error Ryx: " << error[0] << ", 1: " << error[1] << endl;
    Ryx_ = error[0] <= error[1] ? R_yxs[0] : R_yxs[1];


    return true;
}

bool Solver::solveConstraintQyx(Eigen::Vector4d v1, Eigen::Vector4d v2, double &lambda1, double &lambda2)
{
    double a = v1[1] * v1[2] + v1[3] * v1[0];
    double b = v1[1] * v2[2] + v1[2] * v2[1] + v1[3] * v2[0] + v1[0] * v2[3];
    double c = v2[1] * v2[2] + v2[3] * v2[0];

    if (fabs(a) < 1e-10) {
        lambda1 = -c / b;
        lambda2 = -c / b;
        return true;
    }

    double delta2 = b * b - 4 * a * c;

    if (delta2 < 0.0) return false;

    double delta = sqrt(delta2);

    lambda1 = (-b + delta) / (2.0 * a);
    lambda2 = (-b - delta) / (2.0 * a);
    return true;
}

void Solver::correctCamera()
{
    if(sync_result_.size() != cam_datas_.size())
    {
        std::cerr << "ERROR!! correctCamera: sync_result.size() != camDatas.size()" << std::endl;
        return;
    }
    std::vector<data_selection::sync_data> sync_tmp;
    std::vector<data_selection::cam_data> cam_tmp;
    Eigen::Vector3d r3 = Ryx_.row(2);

    for (int i = 0; i < sync_result_.size(); ++i) {
        Eigen::Vector3d tlc_cam = cam_datas_[i].tlc;
        double vcos = tlc_cam.dot(r3) / (r3.norm() * tlc_cam.norm());
        Eigen::Vector3d tlc_cam_p = tlc_cam - (r3 / r3.norm()) * (tlc_cam.norm() * vcos);
        Eigen::Vector3d tlc_corrected = Ryx_ * tlc_cam;
        Eigen::Vector3d tlc_corrected_p = Ryx_ * tlc_cam_p;

        if (i == 0) {
            cout << colouredString("before corrected by Ryx: ", BLUE, REGULAR) << endl;
            cout << tlc_cam << endl;
            cout << colouredString("after corrected by Ryx: ", BLUE, REGULAR) << endl;
            cout << tlc_corrected << endl;
            cout << colouredString("after corrected and projection by Ryx: ", BLUE, REGULAR) << endl;
            cout << tlc_corrected_p << endl;

            cout << endl;
        }
        if (tlc_corrected[2] > 0.1) continue;

        sync_result_[i].scan_match_results[0] = tlc_corrected_p[0];
        sync_result_[i].scan_match_results[1] = tlc_corrected_p[1];
        sync_tmp.push_back(sync_result_[i]);
        cam_tmp.push_back(cam_datas_[i]);
    }
    sync_result_.swap(sync_tmp);
    cam_datas_.swap(cam_tmp);
}

// 求轮速计内参和传感器外参
bool Solver::solve(int max_cond_num)
{
    // First step: estimate J21 and J22
    double J21, J22;

    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    Eigen::Vector2d g = Eigen::Vector2d::Zero();
    Eigen::Vector2d Li = Eigen::Vector2d::Zero();

    // print start infomation
    std::cout << colouredString("-------Start Solve-------", GREEN, BOLD) << std::endl;

    for (int i = 0; i < sync_result_.size(); ++i) {
        const data_selection::sync_data& t = sync_result_[i];
        Li[0] = t.T * t.velocity_left;
        Li[1] = t.T * t.velocity_right;
        
        A = A + Li * Li.transpose();
        g = g + Li * t.scan_match_results[2];
    }

    // Vertify A is not singular
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
    if (cond > max_cond_num) {
        cout << "cond: " << cond << endl;
        std::cout << colouredString("Matrix A is singular", RED, BOLD) << std::endl;
        //return false;
    }

    // y = [J21, J22].T, y = inv(A)g
    Eigen::Vector2d y = Eigen::Vector2d::Zero();
    y = A.colPivHouseholderQr().solve(g);

    std::cout << colouredString("J21=", BLUE, REGULAR) << y(0) << colouredString(", J22=", BLUE, REGULAR) << y(1) << std::endl;

    J21 = y(0); J22 = y(1);

    if (std::isnan(J21) || std::isnan(J22)) {
        std::cout << "cannot find J21 and J22" << std::endl;
        return false;
    }

    // Second Step: estimate the remaining parameters
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd Qi = Eigen::MatrixXd::Zero(2, 5);

    double cx, cy, t1, t2;
    double o_theta;
    for (int i = 0; i < sync_result_.size(); ++i) {
        const data_selection::sync_data& t = sync_result_[i];
        o_theta = t.T * (J21 * t.velocity_left + J22 * t.velocity_right);

        if (fabs(o_theta) > 1e-12) {
            t1 = sin(o_theta) / o_theta;
            t2 = (1 - cos(o_theta)) / o_theta;
        }
        else {
            t1 = 1;
            t2 = 0;
        }

        cx = (-0.5 * J21 * t.T * t.velocity_left + 0.5 * J22 * t.T * t.velocity_right) * t1;
        cy = (-0.5 * J21 * t.T * t.velocity_left + 0.5 * J22 * t.T * t.velocity_right) * t2;

        Qi << -cx, 1-cos(o_theta), sin(o_theta), t.scan_match_results[0], -t.scan_match_results[1],
                -cy, -sin(o_theta), 1-cos(o_theta), t.scan_match_results[1], t.scan_match_results[0];
        M = M + Qi.transpose() * Qi;
    }

    Eigen::VectorXd x = solveConstraintPlane(M);
    res_.b = x(0);
    res_.radius_l = -x(0) * J21;
    res_.radius_r = x(0) * J22;
    res_.l[0] = x(1);
    res_.l[1] = x(2);
    res_.l[2] = atan2(x(4), x(3));

    return true;
}

// 已知轮速计内参，求传感器外参
bool Solver::solve(int max_cond_num, double* r, double b)
{
    double o_theta;
    res_.b = b;
    res_.radius_l = r[0];
    res_.radius_r = r[1];

    double J11 = res_.radius_l / 2;
    double J12 = res_.radius_r / 2;
    double J21 = -res_.radius_l / res_.b;
    double J22 = res_.radius_r / res_.b;

    double t1, t2;
    
    int n = sync_result_.size();
    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(2 * n, 4);
    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(2 * n, 1);
    Eigen::Vector4d m = Eigen::Vector4d::Zero();

    for (int i = 0; i < n; ++i) {
        const data_selection::sync_data& t = sync_result_[i];
        o_theta = t.T * (J21 * t.velocity_left + J22 * t.velocity_right);

        if (fabs(o_theta) > 1e-12) {
            t1 = sin(o_theta) / o_theta;
            t2 = (1 - cos(o_theta)) / o_theta;
        }
        else {
            t1 = 1;
            t2 = 0;
        }

        double prx = (-0.5 * J21 * t.T * t.velocity_left + 0.5 * J22 * t.T * t.velocity_right) * t1 * res_.b;
        double pry = (-0.5 * J21 * t.T * t.velocity_left + 0.5 * J22 * t.T * t.velocity_right) * t2 * res_.b;

        G.block<2, 2>(i * 2, 0) << cos(o_theta) - 1, -sin(o_theta),
                                    sin(o_theta), cos(o_theta) - 1;
        G.block<2, 2>(i * 2, 2) << -t.scan_match_results[0], t.scan_match_results[1],
                                    -t.scan_match_results[1], -t.scan_match_results[0];
        w.block<2, 1>(i * 2, 0) << prx, pry;

    }
    m = G.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-w);
    res_.l[0] = m(0);
    res_.l[1] = m(1);
    res_.l[2] = atan2(m(3), m(2));
    cout << "Gm+w solution: " << "\n" << m << endl;

    return true;
}

Eigen::VectorXd Solver::solveConstraintPlane(const Eigen::MatrixXd& M)
{
    double m11 = M(0, 0);
    double m13 = M(0, 2);
    double m14 = M(0, 3);
    double m15 = M(0, 4);
    double m22 = M(1, 1);
    // double m25 = M(1, 4);
    double m34 = M(2, 3);
    double m35 = M(2, 4);
    double m44 = M(3, 3);
    // double m55 = M(4, 4);
    double a, b, c;

    a = m11 * pow(m22,2) - m22 * pow(m13,2);
    b = 2 * m13 * m22 * m35 * m15 - pow(m22,2) * pow(m15,2) - 2 * m11 * m22 * pow(m35, 2)
        + 2 * m13 * m22 * m34 * m14 - 2 * m22 * pow(m13,2) * m44 - pow(m22,2) * pow(m14,2)
        + 2 * m11 * pow(m22,2) * m44 + pow(m13,2) * pow(m35,2) - 2 * m11 * m22 * pow(m34,2)
        + pow(m13,2) * pow(m34,2);
    c = -2 * m13 * pow(m35, 3) * m15 - m22 * pow(m13,2) * pow(m44,2) + m11 * pow(m22,2) * pow(m44,2)
        + pow(m13,2) * pow(m35,2) * m44 + 2 * m13 * m22 * m34 * m14 * m44
        + pow(m13,2) * pow(m34,2) * m44 - 2 * m11 * m22 * pow(m34,2) * m44
        - 2 * m13 * pow(m34,3) * m14 - 2 * m11 * m22 * pow(m35,2) * m44
        + 2 * m11 * pow(m35,2) * pow(m34,2) + m22 * pow(m14,2) * pow(m35,2)
        - 2 * m13 * pow(m35,2) * m34 * m14 - 2 * m13 * pow(m34, 2) * m35 * m15
        + m11 * pow(m34,4) + m22 * pow(m15,2) * pow(m34,2)
        + m22 * pow(m35,2) * pow(m15,2) + m11 * pow(m35,4)
        - pow(m22,2) * pow(m14,2) * m44 + 2 * m13 * m22 * m35 * m15 * m44
        + m22 * pow(m34,2) * pow(m14,2) - pow(m22,2) * pow(m15,2) * m44;

    double delta = (pow(b, 2) - 4 * a * c);
    if (delta >= 0)
    {
        double r0 = (-b - sqrt(delta)) / (2.0 * a);
        double r1 = (-b + sqrt(delta)) / (2.0 * a);

        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(5, 5);
        W(3, 3) = 1;
        W(4, 4) = 1;
        Eigen::VectorXd x0 = x_given_lambda(M, r0, W);
        Eigen::VectorXd x1 = x_given_lambda(M, r1, W);

        double e0 = x0.transpose() * M * x0;
        double e1 = x1.transpose() * M * x1;

        return e0 < e1 ? x0 : x1;
    }
    else {
        std::cout << "Imaginary solution!" << std::endl;
        return Eigen::VectorXd(5);
    }
    
}

Eigen::VectorXd Solver::x_given_lambda(const Eigen::MatrixXd &M, const double &lambda, const Eigen::MatrixXd &W)
{
  Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(5,5);
  Eigen::MatrixXd ZZ = Eigen::MatrixXd::Zero(5,5);

  Z = M + lambda * W;

  ZZ = Z.transpose() * Z;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(ZZ);

//  Eigen::EigenSolver<Eigen::MatrixXd> es;
//  es.compute(ZZ);

//  Eigen::VectorXd eigenvalues = es.pseudoEigenvalueMatrix();
//  Eigen::MatrixXd eigenvectors = es.pseudoEigenvectors();
  Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues();
  Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors();
  //int colnum = eigenvalues.minCoeff();
  Eigen::VectorXd v0 = eigenvectors.col(0);// min vector
  Eigen::Vector2d tmp_v = Eigen::Vector2d::Zero(2);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Z, Eigen::ComputeFullU | Eigen::ComputeFullV);
  v0 = svd.matrixV().col(4);
  tmp_v(0) = v0(3);
  tmp_v(1) = v0(4);

  double norm = tmp_v.norm();
  double coeff = (v0(0) >= 0 ? 1 : -1) / norm;
  v0 = coeff * v0;
  return v0;
}

void Solver::calibrate(int outliers_iteration, int sensor, double* r, double b)
{
    std::vector<data_selection::sync_data> outliers_data;
    // print start calibrate infomation
    std::cout << colouredString("-------Start Calibrate-------", GREEN, BOLD) << std::endl;

    if (sensor != 1) {
        estimateRyx();
        correctCamera();
    }
    
    for (int i = 0; i < outliers_iteration; ++i) {
        bool ok;
        if (r == nullptr) ok = solve(75);
        else ok = solve(75, r, b);
        if (!ok) {
            std::cout << "Failed calibration" << std::endl;
            continue;
        }
        
        // compute residuals
        for (int i = 0; i < sync_result_.size(); ++i) 
            computeDisagreement(sync_result_[i]);

        std::vector<double> err_theta, err_xy;
        for (int i = 0; i < sync_result_.size(); ++i) {
            double x = sync_result_[i].err[0];
            double y = sync_result_[i].err[1];
            double theta = sync_result_[i].err[2];

            err_theta.push_back(fabs(theta));
            err_xy.push_back(hypot(x, y));
        }

        std::vector<double> err_theta_sorted(err_theta);
        std::vector<double> err_xy_sorted(err_xy);
        
        std::sort(err_theta_sorted.begin(), err_theta_sorted.end());
        std::sort(err_xy_sorted.begin(), err_xy_sorted.end());

        int threshold_index = (int) std::round(0.9 * sync_result_.size());
        double threshold_theta = err_theta_sorted[threshold_index];
        double threshold_xy = err_xy_sorted[threshold_index];

        std::vector<data_selection::sync_data> n;
        for (int i = 0; i < sync_result_.size(); ++i) {
            bool xy = err_xy[i] > threshold_xy;
            bool theta = err_theta[i] > threshold_theta;

            if (!(xy | theta)) 
                n.push_back(sync_result_[i]);
            else outliers_data.push_back(sync_result_[i]);
        }

        sync_result_ = n;
    }

    if (sensor != 1)
        estimateRyx();
    refineExParam();

    std::cout << '\n' << colouredString("-------Calibration Results-------", GREEN, BOLD) << '\n' 
                 << colouredString("Axle between wheels: ", RED, BOLD) << res_.b << '\n' 
                 << colouredString("cam-odom x: ", RED, BOLD) << res_.l[0] << '\n'
                 << colouredString("cam-odom y: ", RED, BOLD) << res_.l[1] << '\n' 
                 << colouredString("Left wheel radius: ", RED, BOLD) << res_.radius_l << '\n'
                 << colouredString("Right wheel radius: ", RED, BOLD) << res_.radius_r  << std::endl;
    cout << colouredString("estimate Roc: ", RED, BOLD) << "\n" << res_.Toc.block<3, 3>(0, 0).matrix() << endl;

    return;
}

void Solver::computeDisagreement(data_selection::sync_data& calib_data)
{
    double J11 = res_.radius_l / 2;
    double J12 = res_.radius_r / 2;
    double J21 = -res_.radius_l / res_.b;
    double J22 = res_.radius_r / res_.b;

    double v = J11 * calib_data.velocity_left + J12 * calib_data.velocity_right;
    double omega = J21 * calib_data.velocity_left + J22 * calib_data.velocity_right;

    double o_theta = calib_data.T * omega;

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

    // 轮式里程计计算的里程计数据
    calib_data.o[0] = t1 * v * calib_data.T;
    calib_data.o[1] = t2 * v * calib_data.T;
    calib_data.o[2] = o_theta;

    // l: 相机外参，s: 相机里程计，o: 轮式里程计
    double l_plus_s[3];
    double o_plus_l[3];

    oplus_d(res_.l, calib_data.scan_match_results, l_plus_s);
    oplus_d(calib_data.o, res_.l, o_plus_l);

    //error = (l+s) - (o+l)
    for (int i = 0; i < 3; ++i)
        calib_data.err[i] = l_plus_s[i] - o_plus_l[i];

    // 轮式里程计和相机外参估计相机里程计：est_sm = -l + s + l (8)
    pose_diff_d(o_plus_l, res_.l, calib_data.est_sm);    

    for (int i = 0; i < 3; ++i)
        calib_data.err_sm[i] = calib_data.est_sm[i] - calib_data.scan_match_results[i];
}

// 优化外参
void Solver::refineExParam()
{
    std::vector<Eigen::Quaterniond> q_cam, q_odo;
    std::vector<Eigen::Vector3d> t_cam, t_odo;

    double J11 = res_.radius_l / 2;
    double J12 = res_.radius_r / 2;
    double J21 = -res_.radius_l / res_.b;
    double J22 = res_.radius_r / res_.b;

    Eigen::Vector2d toc;
    toc << res_.l[0], res_.l[1];

    for (int i = 0;i < sync_result_.size(); ++i) {
        q_cam.push_back(sync_result_[i].qcl_cam);
        t_cam.push_back(sync_result_[i].tcl_cam);

        
        double v = J11 * sync_result_[i].velocity_left + J12 * sync_result_[i].velocity_right;
        double omega = J21 * sync_result_[i].velocity_left + J22 * sync_result_[i].velocity_right;
        
        Eigen::Quaterniond qlc_odo;
        Eigen::Vector3d tlc_odo;

        double o_theta = sync_result_[i].T * omega;

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

        qlc_odo = Eigen::Quaterniond(Eigen::AngleAxisd(o_theta, Eigen::Vector3d::UnitZ()));
        tlc_odo << v * sync_result_[i].T * t1, v * sync_result_[i].T * t2, 0;

        q_odo.push_back(qlc_odo.inverse());
        t_odo.push_back(-qlc_odo.matrix().inverse() * tlc_odo);
    }

    Eigen::Matrix3d Roc = Eigen::AngleAxisd(res_.l[2], Eigen::Vector3d::UnitZ()) * Ryx_;

    Eigen::Matrix4d Toc = Eigen::Matrix4d::Identity();
    Toc.block<3, 3>(0, 0) = Roc;
    Toc.block<2, 1>(0, 3) = toc;

    refineEstimate(Toc, 1.0, q_odo, t_odo, q_cam, t_cam);

    res_.l[0] = Toc(0, 3);
    res_.l[1] = Toc(1, 3);
    res_.Toc = Toc;
}

// 优化估计
void Solver::refineEstimate(Eigen::Matrix4d &Toc, double scale,
    const std::vector<Eigen::Quaterniond > &quats_odo,
    const std::vector<Eigen::Vector3d> &tvecs_odo,
    const std::vector<Eigen::Quaterniond> &quats_cam,
    const std::vector<Eigen::Vector3d> &tvecs_cam)
  {
   Eigen::Quaterniond q(Toc.block<3,3>(0,0));
   double q_coeffs[4] = {q.w(),q.x(),q.y(),q.z()};
   double t_coeffs[3] = {Toc(0,3),Toc(1,3),Toc(2,3)};
   ceres::Problem problem;
   for(size_t i = 0; i< quats_odo.size(); ++i)
   {
    ceres::CostFunction * costfunction =
    new ceres::AutoDiffCostFunction<CameraOdomErr, 7,4,3>(
            new CameraOdomErr(quats_odo.at(i) , tvecs_odo.at(i), quats_cam.at(i) , tvecs_cam.at(i) ) );   //  residual : 6 ,  rotation: 4

#ifdef LOSSFUNCTION
      //ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
    ceres::LossFunction * loss_function = new ceres::HuberLoss(1.0);
    problem.AddResidualBlock(costfunction, loss_function, q_coeffs, t_coeffs);
#else
    problem.AddResidualBlock(costfunction, NULL, q_coeffs, t_coeffs);
#endif

  }
  ceres::LocalParameterization* quaternionParameterization = new ceres::QuaternionParameterization;
  problem.SetParameterization(q_coeffs,quaternionParameterization);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 100;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, & summary);
  q = Eigen::Quaterniond(q_coeffs[0],q_coeffs[1],q_coeffs[2],q_coeffs[3]);

  Toc.block<3,3>(0,0) = q.toRotationMatrix();
  Toc.block<3,1>(0,3) << t_coeffs[0],t_coeffs[1],t_coeffs[2];
}