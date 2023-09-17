#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include <fstream>
#include <Eigen/Dense>  

class data_selection
{
public:
    int sensor_;
    struct odo_data
    {
        double time;
        double v_left;
        double v_right;
    };
    struct cam_data
    {
        double start_t;
        double end_t;
        //double theta_y;
        // 这里的deltaTheta的旋转轴需要垂直于地面向上，因为轮速计的旋转轴垂直于地面向上，这样deltaTheta大小才相等
        double deltaTheta;
        Eigen::Vector3d axis;
        Eigen::Matrix3d Rcl;
        Eigen::Vector3d tlc; 
    };
    /*
   * \brief The sync_data struct. Used for
   * storing synchronized data.
   */
  struct sync_data {
    // Period
    double T;
    // Left and right wheel velocities
    double velocity_left;
    double velocity_right;
    // double velocity;
    //camera data : x y yaw , x  y from tlc (not tcl)
    double scan_match_results[3];// correct lx ly by R_x
    // Estimated rototranslation based on odometry params.
    double o[3];
    // Estimated disagreement  sm - est_sm
    double est_sm[3];
    double err_sm[3]; //  s  - (-) l (+) o (+) l
    // Other way to estimate disagreement:   l (+) s  - o (+) l
    double err[3];
    int mark_as_outlier;
    //tcl_cam and qcl_cam are original data(not correted by R_x)
    Eigen::Vector3d tcl_cam;//06/06  
    Eigen::Quaterniond qcl_cam;
    // cl
    double angle;
    Eigen::Vector3d axis;
    double startTime;

  };

    void selectData(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,
        std::vector<data_selection::sync_data> &sync_result);

    data_selection(int sensor): sensor_(sensor) {}

    
private:
    void startPosAlign(std::vector<odo_data>& odoDatas, std::vector<cam_data>& camDatas);
    void camOdoAlign(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,std::vector<sync_data> &sync_result);
};

// 将轮速计数据和相机里程计数据的起始时刻对齐
void data_selection::startPosAlign(std::vector<odo_data>& odoDatas, std::vector<cam_data>& camDatas)
{
    double t0_cam = camDatas[0].start_t;
    double t0_odo = odoDatas[0].time;
    if(t0_cam > t0_odo)
    {
        double delta_t = fabs(odoDatas[0].time - t0_cam);

        int odo_start = 0;
        while(fabs(odoDatas[++odo_start].time - t0_cam) < delta_t) delta_t = fabs(odoDatas[odo_start].time - t0_cam);
        int odo_aligned = odo_start - 1;
        for (int i = 0; i < odo_aligned; ++i)
            odoDatas.erase(odoDatas.begin());

        std::cout << std::fixed << "aligned start position1: "<< camDatas[0].start_t << "  " << odoDatas[0].time <<std::endl;
        return;
    }
    else if(t0_odo > t0_cam)
    {
        double delta_t = fabs(camDatas[0].start_t - t0_odo);

        int cam_start = 0;
        while(fabs(camDatas[++cam_start].start_t - t0_odo) < delta_t) delta_t = fabs(camDatas[cam_start].start_t - t0_odo);
        int cam_aligned = cam_start - 1;
        for (int i = 0; i < cam_aligned; ++i)
            camDatas.erase(camDatas.begin());

        std::cout << std::fixed << "aligned start position2 : "<< camDatas[0].start_t << "  " << odoDatas[0].time <<std::endl;
        return;
    }
    else
        return;
}

// 对每一段相机里程计，找到对应时间的一组轮速计数据并积分得到相应的轮式里程计，时间点对不上的轮速计数据进行插值处理
// 保存N段对应的相机里程计和轮式里程计数据
void data_selection::camOdoAlign(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,std::vector<sync_data> &sync_result)
{
    int id_odo = 2;
    std::vector<data_selection::odo_data> odoDatas_tmp;
    std::vector<data_selection::cam_data> camDatas_tmp;
    std::cout << "canOdoAlign start" << std::endl;
    // 这里从3开始舍弃首末3个数据是因为方便后续找cam和odom首末时间对齐以及odom插值
    for (int i = 3; unsigned(i) < camDatas.size() - 3; ++i)
    {
        // 相机转动数据无效、平移过小以及转动轴和已知（依据相机坐标系）相差过大，这些数据不作考虑
        if( std::isnan(camDatas[i].deltaTheta) )
            continue;

        double tlc_length = camDatas[i].tlc.norm();
        if(tlc_length < 1e-4)
            continue;       
        if (sensor_ == 0 && camDatas[i].axis(1) > -0.9)
            continue;
        if (sensor_ == 2 && camDatas[i].axis(2) < 0.9)
            continue;

        data_selection::odo_data   odo_start, odo_end;
        id_odo -= 2;
        double t_interval = fabs(odoDatas[id_odo].time - camDatas[i].start_t);
        // 找到离该段相机帧间运动开始时间最近的odom数据
        while(fabs(odoDatas[++id_odo].time - camDatas[i].start_t) < t_interval) {
            t_interval = fabs(odoDatas[id_odo].time - camDatas[i].start_t);
        } 
        int odo_start_id = id_odo - 1;
        odo_start = odoDatas[odo_start_id];

        id_odo -= 2;
        t_interval = fabs(odoDatas[id_odo].time - camDatas[i].end_t);
        // 找到离该段相机帧间运动结束时间最近的odom数据
        while(fabs(odoDatas[++id_odo].time - camDatas[i].end_t) < t_interval) {
            t_interval = fabs(odoDatas[id_odo].time - camDatas[i].end_t);
        }
            
        int odo_end_id = id_odo - 1;
        odo_end = odoDatas[odo_end_id];

        if(odo_end_id - odo_start_id == 0)
            continue;

        // odo data: insert value
        double v_left_start , v_left_end;
        double v_right_start, v_right_end;
        
        // 轮速计对齐开始时刻数据线性插值
        if(camDatas[i].start_t > odo_start.time)
        {
            float alpha = (camDatas[i].start_t - odoDatas[odo_start_id].time)
            / (odoDatas[odo_start_id+1].time - odoDatas[odo_start_id].time);
            alpha = fabs(alpha);
            v_left_start = (1-alpha) * odoDatas[odo_start_id].v_left + alpha * odoDatas[odo_start_id+1].v_left; 
            v_right_start =  (1-alpha) * odoDatas[odo_start_id].v_right + alpha * odoDatas[odo_start_id+1].v_right; 
        }
        else if(camDatas[i].start_t < odo_start.time)
        {
            float alpha = (odoDatas[odo_start_id].time - camDatas[i].start_t)
            / (odoDatas[odo_start_id].time - odoDatas[odo_start_id-1].time);
            alpha = fabs(alpha);
            v_left_start = (1-alpha) * odoDatas[odo_start_id].v_left + alpha * odoDatas[odo_start_id-1].v_left;
            v_right_start =  (1-alpha) * odoDatas[odo_start_id].v_right + alpha * odoDatas[odo_start_id-1].v_right;
        }
        else
        {
            v_left_start = odo_start.v_left;
            v_right_start = odo_start.v_right;
        }
        // 将对齐的起始轮速计数据修改为插值后的数据
        odoDatas[odo_start_id].time = camDatas[i].start_t;
        odoDatas[odo_start_id].v_left = v_left_start;
        odoDatas[odo_start_id].v_right = v_right_start;

        // 轮速计对齐结束时刻数据线性插值
        if(camDatas[i].end_t > odo_end.time)
        {
            float alpha = (camDatas[i].end_t - odoDatas[odo_end_id].time)
            / (odoDatas[odo_end_id+1].time - odoDatas[odo_end_id].time);
            alpha = fabs(alpha);
            v_left_end = (1-alpha) * odoDatas[odo_end_id].v_left + alpha * odoDatas[odo_end_id+1].v_left;
            v_right_end =  (1-alpha) * odoDatas[odo_end_id].v_right + alpha * odoDatas[odo_end_id+1].v_right;
        }
        else if(camDatas[i].end_t < odo_end.time)
        {
            float alpha = (odoDatas[odo_end_id].time - camDatas[i].end_t)
            / (odoDatas[odo_end_id].time - odoDatas[odo_end_id-1].time);
            alpha = fabs(alpha);
            v_left_end = (1-alpha) * odoDatas[odo_end_id].v_left + alpha * odoDatas[odo_end_id-1].v_left;
            v_right_end =  (1-alpha) * odoDatas[odo_end_id].v_right + alpha * odoDatas[odo_end_id-1].v_right;
        }
        else
        {
            v_left_end = odo_end.v_left;
            v_right_end = odo_end.v_right;
        }
        odoDatas[odo_end_id].time = camDatas[i].end_t;
        odoDatas[odo_end_id].v_left = v_left_end;
        odoDatas[odo_end_id].v_right = v_right_end;

        //get the average ang_vel and lin_vel between camDatas[i].start_t and camDatas[i].end_t
        data_selection::odo_data   odo_tmp;//odo_tmp
        odo_tmp.time = odoDatas[odo_start_id].time;//odo_tmp
        data_selection::sync_data sync_tmp;
        if(odo_end_id - odo_start_id > 1)
        {
            double dis_left_sum = 0.0, dis_right_sum = 0.0;
            // 轮速计数据中值积分
            for(int j = odo_start_id; j < odo_end_id; j++)
            {
                dis_left_sum += (odoDatas[j+1].time - odoDatas[j].time) * (odoDatas[j+1].v_left + odoDatas[j].v_left) / 2.0;
                dis_right_sum += (odoDatas[j+1].time - odoDatas[j].time) * (odoDatas[j+1].v_right + odoDatas[j].v_right) / 2.0;
            }
            // 这里取均值是因为后续计算只需轮速计原始数据积分，之后求xT即可
            double T = camDatas[i].end_t - camDatas[i].start_t;
            odo_tmp.v_left = dis_left_sum / T;
            odo_tmp.v_right = dis_right_sum / T;
            
            sync_tmp.T = T; 
            sync_tmp.velocity_left = dis_left_sum / T;
            sync_tmp.velocity_right = dis_right_sum / T;
            
               // 1: robot      camera    robot      camera
               //         x               z        |            x              x         
                //        y               -x      |            y              z 
                //       z               y         |            z               y              

            // 平面数据
            sync_tmp.scan_match_results[0] = camDatas[i].tlc[0];
            sync_tmp.scan_match_results[1] = camDatas[i].tlc[1];
            sync_tmp.scan_match_results[2] = camDatas[i].deltaTheta;

            sync_tmp.tcl_cam = -camDatas[i].Rcl * camDatas[i].tlc;
            sync_tmp.qcl_cam = Eigen::Quaterniond(camDatas[i].Rcl);
            sync_tmp.angle = -camDatas[i].deltaTheta; // should be cl
            sync_tmp.axis = camDatas[i].axis;
            sync_tmp.startTime = camDatas[i].start_t;

        }
        else if(odo_end_id - odo_start_id == 1)
        {   
            double T = camDatas[i].end_t - camDatas[i].start_t;
            double ave_v_left = (v_left_start + v_left_end) / 2.0;
            double ave_v_right = (v_right_start + v_right_end) / 2.0;

            odo_tmp.v_left = ave_v_left;
            odo_tmp.v_right = ave_v_right;

            sync_tmp.T = T;
            sync_tmp.velocity_left = ave_v_left;
            sync_tmp.velocity_right = ave_v_right;
            
              // 1: robot      camera    robot      camera
               //         x               z        |            x              x         
                //        y               -x      |            y              z 
                //       z               y         |            z               y

            sync_tmp.scan_match_results[0] = camDatas[i].tlc[0];
            sync_tmp.scan_match_results[1] = camDatas[i].tlc[1];
            //double angle_tmp = (camDatas[i].theta_y > 0.0?1:(-1)) * camDatas[i].deltaTheta;
            sync_tmp.scan_match_results[2] = camDatas[i].deltaTheta;

            sync_tmp.tcl_cam = -camDatas[i].Rcl * camDatas[i].tlc;
            sync_tmp.qcl_cam = Eigen::Quaterniond(camDatas[i].Rcl);
            sync_tmp.angle = -camDatas[i].deltaTheta; // should be cl
            sync_tmp.axis = camDatas[i].axis;
            sync_tmp.startTime = camDatas[i].start_t;
        
        }

        camDatas_tmp.push_back(camDatas[i]);
        odoDatas_tmp.push_back(odo_tmp);//odo_tmp
        sync_result.push_back(sync_tmp);

        id_odo--;
    }
    camDatas.swap(camDatas_tmp);
    odoDatas.swap(odoDatas_tmp);
}

// 对齐并去掉一些不需要的数据
void data_selection::selectData(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,
        std::vector<data_selection::sync_data> &sync_result)
{
    startPosAlign(odoDatas,camDatas);
    //get rid of cam data ( deltaTheta = nan, tlc_length<1e-4,axis(1)<0.96), and align cam and odo data
    camOdoAlign(odoDatas,camDatas,sync_result);  
    std::cout << "can odo align" << std::endl;
    //get rid of the odo data whose avg velocity is less than 1e-4 and whose theta sign isn't same
    std::vector<data_selection::sync_data> vec_sync_tmp;
    std::vector<odo_data> odo_matches;
    std::vector<cam_data> cam_matches;
    int size = std::min(camDatas.size(), odoDatas.size());

    int nFlagDiff = 0;
    for (int i = 0; i < size; ++i)
    {
        if(fabs(odoDatas[i].v_left) < 1e-3 || fabs(odoDatas[i].v_right) < 1e-3)
            continue;
        //the sign is opposite
        if((odoDatas[i].v_right - odoDatas[i].v_left) * camDatas[i].deltaTheta < 0.0)// rL == rR
            {nFlagDiff++; continue; }
        odo_matches.push_back(odoDatas[i]);
        cam_matches.push_back(camDatas[i]);
        vec_sync_tmp.push_back(sync_result[i]);
    }
    std::cout << "nFlagDiff = " << nFlagDiff <<std::endl;
    sync_result.swap(vec_sync_tmp);
    camDatas.swap(cam_matches);
    odoDatas.swap(odo_matches);

}
