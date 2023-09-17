
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
    
    prevTime = -1;
    curTime = 0;
    // openExEstimation = 0;

    initP = Vector3d(0,0,0);
    initR = Matrix3d::Identity();

    inputOdomCnt = 0;

    // inputImageCnt = 0;
    initFirstPoseFlag = false;

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

    first_imu = false;
    // sum_of_back = 0;
    // sum_of_front = 0;
    frame_count = 0;
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

// 删除t0时刻之前的IMU数据
// 将t0~t1时刻之间的IMU数据从buf中弹出，加入accVector、gyrVector中，及添加buff中队列头一帧数据
bool Estimator::getIMUInterval(double t0, double t1, vector<pair<double, Eigen::Vector3d>> &accVector, 
                                vector<pair<double, Eigen::Vector3d>> &gyrVector)
{
    if (accBuf.empty())
    {
        printf("not receive imu\n");
        return false;
    }
    //printf("get imu from %f %f\n", t0, t1);
    //printf("imu fornt time %f   imu end time %f\n", accBuf.front().first, accBuf.back().first);
    if(t1 <= accBuf.back().first)
    {
        while (accBuf.front().first <= t0) 
        {
            accBuf.pop();
            gyrBuf.pop();
        }
        while (accBuf.front().first < t1) {
            accVector.push_back(accBuf.front());
            accBuf.pop();
            gyrVector.push_back(gyrBuf.front());
            gyrBuf.pop();
        }
        accVector.push_back(accBuf.front());
        gyrVector.push_back(gyrBuf.front());
    }
    else
    {
        printf("wait for imu\n");
        return false;
    }
    return true;
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
            odomBuf.pop();
            // featureBuf.pop();
            mBuf.unlock();
            if (USE_IMU)
            {
                if(!initFirstPoseFlag)
                    initFirstIMUPose(accVector);
                for(size_t i = 0; i < accVector.size(); i++)
                {
                    double dt;
                    if(i == 0) 
                        dt = accVector[i].first - prevTime;
                    else if (i == accVector.size() - 1)
                        dt = curTime - accVector[i - 1].first;
                    else
                        dt = accVector[i].first - accVector[i - 1].first;
                    processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
                }
            }

            mProcess.lock();
            processOdom(odom.first, odom.second);
            // processImage(feature.second, feature.first);
            prevTime = curTime;

            // printStatistics(*this, 0);

            // std_msgs::Header header;
            // header.frame_id = "world";
            // header.stamp = ros::Time(feature.first);

            // pubOdometry(*this, header);
            // pubKeyPoses(*this, header);
            // pubCameraPose(*this, header);
            // pubPointCloud(*this, header);
            // pubKeyframe(*this);
            // pubTF(*this, header);
            // mProcess.unlock();
        }

        if (! MULTIPLE_THREAD)
            break;

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

// 
void Estimator::initFirstIMUPose(vector<pair<double, Eigen::Vector3d>> &accVector)
{
    printf("init first imu pose\n");
    initFirstPoseFlag = true;
    // 先求一个平均值
    Eigen::Vector3d averAcc(0, 0, 0);
    int n = (int)accVector.size();
    for(size_t i = 0; i < accVector.size(); i++)
    {
        averAcc = averAcc + accVector[i].second;
    }
    averAcc = averAcc / n;
    printf("averge acc %f %f %f\n", averAcc.x(), averAcc.y(), averAcc.z());

    Matrix3d R0 = Utility::g2R(averAcc);
    double yaw = Utility::R2ypr(R0).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;

    // first imu pose
    Rs[0] = R0;
    cout << "init R0 " << endl << Rs[0] << endl;
}

void Estimator::processIMU(double t, double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    if(!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        // 将当次观测加入到IMU预积分中
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        // 计算世界坐标系中的线加速度、角速度值 (旋转变换前)
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        // 计算旋转矩阵变换后的值
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        // 计算世界坐标系中的线加速度 (旋转变换后)，平均线加速度
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        // 计算平移量、速度
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processOdom(double t, const Matrix3d &odom)
{
    // ROS_DEBUG("new image coming ------------------------------------------");

    // 检查是否新帧是否加入关键帧
    // if (f_manager.addFeatureCheckParallax(frame_count, image, td))
    // {
    //     marginalization_flag = MARGIN_OLD;
    //     //printf("keyframe\n");
    // }
    // else
    // {
    //     marginalization_flag = MARGIN_SECOND_NEW;
    //     //printf("non-keyframe\n");
    // }

    // ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    // ROS_DEBUG("Solving %d", frame_count);
    // ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    // Headers[frame_count] = header;

    // ImageFrame imageframe(image, header);
    // imageframe.pre_integration = tmp_pre_integration;
    // all_image_frame.insert(make_pair(header, imageframe));
    // tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        // ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                // ROS_WARN("initial extrinsic rotation calib success");
                // ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    // if (solver_flag == INITIAL)
    // {
    //     // monocular + IMU initilization
    //     if (!STEREO && USE_IMU)
    //     {
    //         if (frame_count == WINDOW_SIZE)
    //         {
    //             bool result = false;
    //             if(ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
    //             {
    //                 result = initialStructure();
    //                 initial_timestamp = header;   
    //             }
    //             if(result)
    //             {
    //                 optimization();
    //                 updateLatestStates();
    //                 solver_flag = NON_LINEAR;
    //                 slideWindow();
    //                 ROS_INFO("Initialization finish!");
    //             }
    //             else
    //                 slideWindow();
    //         }
    //     }

    //     if(frame_count < WINDOW_SIZE)
    //     {
    //         frame_count++;
    //         int prev_frame = frame_count - 1;
    //         Ps[frame_count] = Ps[prev_frame];
    //         Vs[frame_count] = Vs[prev_frame];
    //         Rs[frame_count] = Rs[prev_frame];
    //         Bas[frame_count] = Bas[prev_frame];
    //         Bgs[frame_count] = Bgs[prev_frame];
    //     }

    // }
    // else
    // {
    //     TicToc t_solve;
    //     if(!USE_IMU)
    //         f_manager.initFramePoseByPnP(frame_count, Ps, Rs, tic, ric);
    //     f_manager.triangulate(frame_count, Ps, Rs, tic, ric);
    //     optimization();
    //     set<int> removeIndex;
    //     outliersRejection(removeIndex);
    //     f_manager.removeOutlier(removeIndex);
    //     if (! MULTIPLE_THREAD)
    //     {
    //         featureTracker.removeOutliers(removeIndex);
    //         predictPtsInNextFrame();
    //     }
            
    //     ROS_DEBUG("solver costs: %fms", t_solve.toc());

    //     if (failureDetection())
    //     {
    //         ROS_WARN("failure detection!");
    //         failure_occur = 1;
    //         clearState();
    //         setParameter();
    //         ROS_WARN("system reboot!");
    //         return;
    //     }

    //     slideWindow();
    //     f_manager.removeFailures();
    //     // prepare output of VINS
    //     key_poses.clear();
    //     for (int i = 0; i <= WINDOW_SIZE; i++)
    //         key_poses.push_back(Ps[i]);

    //     last_R = Rs[WINDOW_SIZE];
    //     last_P = Ps[WINDOW_SIZE];
    //     last_R0 = Rs[0];
    //     last_P0 = Ps[0];
    //     updateLatestStates();
    // }
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
