
#include "estimator/estimator.h"

Estimator::Estimator()
{
    // ROS_INFO("Initialization begins");
    initThreadFlag = false;
    clearState();
}

// Estimator::~Estimator()
// {
//     if (MULTIPLE_THREAD)
//     {
//         processThread.join();
//         printf("join thread \n");
//     }
// }

void Estimator::clearState()
{
    mProcess.lock();
    while(!accBuf.empty())
        accBuf.pop();
    while(!gyrBuf.empty())
        gyrBuf.pop();
    
    // prevTime = -1;
    // curTime = 0;
    // openExEstimation = 0;

    initP = Vector3d(0,0,0);
    initR = Matrix3d::Identity();

    inputOdomCnt = 0;

    // inputImageCnt = 0;
    // initFirstPoseFlag = false;

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
        {
            delete pre_integrations[i];
        }
        pre_integrations[i] = nullptr;
    }

    // for (int i = 0; i < NUM_OF_CAM; i++)
    // {
    //     tic[i] = Vector3d::Zero();
    //     ric[i] = Matrix3d::Identity();
    // }

    // first_imu = false,
    // sum_of_back = 0;
    // sum_of_front = 0;
    // frame_count = 0;
    // solver_flag = INITIAL;
    // initial_timestamp = 0;
    // all_image_frame.clear();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    // if (last_marginalization_info != nullptr)
    //     delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    // last_marginalization_info = nullptr;
    // last_marginalization_parameter_blocks.clear();
    // f_manager.clearState();

    // failure_occur = 0;

    mProcess.unlock();
}

void Estimator::setParameter()
{
    mProcess.lock();
    // for (int i = 0; i )

    td = TD;
    g = G;
    cout << "set g " << g.transpose() << endl;

    std::cout << "MULTIPLE_THREAD is " << MULTIPLE_THREAD << '\n';
    if (MULTIPLE_THREAD && !initThreadFlag)
    {
        initThreadFlag = true;
        processThread = std::thread(&Estimator::processMeasurements, this);
    }
    mProcess.unlock();

}

void Estimator::inputOdm(double t, const Matrix3d &odom)
{
    inputOdomCnt++;
    odomBuf.push(make_pair(t, odom));

    TicToc processTime;
    processMeasurements();
    printf("process time: %f\n", processTime.toc());
}

void Estimator::inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
{
    mBuf.lock();
    accBuf.push(make_pair(t, linearAcceleration));
    gyrBuf.push(make_pair(t, angularVelocity));
    //printf("input imu with time %f \n", t);
    mBuf.unlock();

    // if (solver_flag == NON_LINEAR)
    // {
    //     mPropagate.lock();
    //     fastPredictIMU(t, linearAcceleration, angularVelocity);
    //     pubLatestOdometry(latest_P, latest_Q, latest_V, t);
    //     mPropagate.unlock();
    // }

}

// 判断是否已经获得了截止到某个时刻的IMU数据
bool Estimator::IMUAvailable(double t)
{
    if(!accBuf.empty() && t <= accBuf.back().first)
        return true;
    else
        return false;
}


void Estimator::processMeasurements()
{
    while(1)
    {
        pair<double, Matrix3d> odom;
        vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
        if (!odomBuf.empty())
        {
            odom = odomBuf.front();
            curTime = odom.first + td;
            while(1){
                if (!USE_IMU || IMUAvailable(odom.first+td))
                    break;
                else{
                    printf("wait for imu ... \n");
                    if (!MULTIPLE_THREAD)
                        return;
                    std::chrono::milliseconds dura(5);
                    std::this_thread::sleep_for(dura);
                }

            }
            mBuf.lock();
            if(USE_IMU){
                getIMUInterval(prevTime, curTime, accVector, gyrVector);
            }
        }
    }
}


void Estimator::optimization()
{
    // ceres::Problem problem;
    // ceres::LossFunction *loss_function;

    // loss_function = new ceres::HuberLoss(1.0);

    // // 遍历已有帧，

    // if (USE_IMU)
    // {
    //     for (int i = 0; i < frame_count;)
    // }

}
