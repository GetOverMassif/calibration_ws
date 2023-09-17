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


    // internal
    void clearState();
    void setParameter();
    bool IMUAvailable(double t);
    void processMeasurements();

    void optimization();

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


    SolverFlag solver_flag;


    Vector3d        Ps[(WINDOW_SIZE + 1)];
    Vector3d        Vs[(WINDOW_SIZE + 1)];
    Matrix3d        Rs[(WINDOW_SIZE + 1)];
    Vector3d        Bas[(WINDOW_SIZE + 1)];
    Vector3d        Bgs[(WINDOW_SIZE + 1)];
    double td;


    int inputOdomCnt;

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    IntegrationBase *tmp_pre_integration;

    bool initThreadFlag;

};


