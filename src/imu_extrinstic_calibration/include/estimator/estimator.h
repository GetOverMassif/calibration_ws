#include <thread>
#include <mutex>
#include <queue>

#include <std_msgs/msg/header.hpp>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "ceres/ceres.h"

#include "estimator/parameters.h"
#include "factor/integration_base.h"
#include "utility/tic_toc.h"

using namespace Eigen;
using namespace std;


class Estimator
{
public:
    Estimator();
    ~Estimator();

    // interface
    void inputOdm(double t, const Matrix3d &odom);
    void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity);

    void processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processOdom(double t, const Matrix3d &odom);

    // internal
    void clearState();
    void setParameter();
    bool IMUAvailable(double t);

    void initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector);
    void processMeasurements();

    void optimization();

    bool getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                              vector<pair<double, Eigen::Vector3d>> &gyrVector);

    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    std::mutex mProcess;
    std::mutex mBuf;
    std::mutex mPropagate;
    queue<pair<double, Eigen::Vector3d>> accBuf;
    queue<pair<double, Eigen::Vector3d>> gyrBuf;

    queue<pair<double, Eigen::Matrix3d>> odomBuf;

    double prevTime, curTime;  // 用来记录图像帧（或里程计）前一帧和当前帧时间


    std::thread processThread; // 用来循环处理观测数据的线程

    SolverFlag solver_flag;
    Vector3d g;


    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    int frame_count;

    bool first_imu;


    int inputOdomCnt;

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    IntegrationBase *tmp_pre_integration;

    // 初始 Camera-IMU 外参估计
    Eigen::Vector3d initP;
    Eigen::Matrix3d initR;

    bool initFirstPoseFlag; // 是否已经初始化IMU起始姿态
    bool initThreadFlag;  // 是否在初始化线程
};
