#pragma once

#include <iostream>
#include <vector>

#include "Eigen/Dense"


class PointCloudAlign
{
public:
    PointCloudAlign(int mode): mode_(mode) {}

    Eigen::Matrix3d align2d(const std::vector<Eigen::Vector2d>& points_global, const std::vector<Eigen::Vector2d>& points_base,
                            const std::vector<int>& matched_gb);

    Eigen::Vector2d threeSidedPositioning(const std::vector<Eigen::Vector2d>& points_global, const std::vector<Eigen::Vector2d>& points_base,
                            const std::vector<int>& matched_gb);
    
private:
    // 0: calculate the position by point cloud centroid, 1: caculate the position by three-sided positioning
    int mode_ = 0;
};

Eigen::Matrix3d PointCloudAlign::align2d(const std::vector<Eigen::Vector2d>& points_global, const std::vector<Eigen::Vector2d>& points_base,
                        const std::vector<int>& matched_gb) {
    // error if the number of points is not equal
    if (points_global.size() != points_base.size()) {
        std::cerr << "The number of points is not equal!" << std::endl;
        return Eigen::Matrix3d::Identity();
    }
    
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
    // calculate the centroid of each point set
    Eigen::Vector2d centroid_global = Eigen::Vector2d::Zero();
    Eigen::Vector2d centroid_base = Eigen::Vector2d::Zero();
    for (int i = 0; i < points_global.size(); ++i) {
        centroid_global += points_global[i];
        centroid_base += points_base[i];
    }
    centroid_global /= points_global.size();
    centroid_base /= points_base.size();

    // calculate the covariance matrix
    // 计算协方差矩阵
    // TODO: 此处数学原理/*  */
    Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
    for (int i = 0; i < points_base.size(); ++i) {
        W += (points_base[matched_gb[i]] - centroid_base) * (points_global[i] - centroid_global).transpose();
    }
    W /= points_global.size();

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    // different situations of det(W)
    // simplify here
    double det_W = W.determinant();
    Eigen::Matrix2d R;
    Eigen::Vector2d t;
    if (det_W > 0) {
        R = U * V.transpose();
    }
    else {
        std::cerr << "det(W) < 0" << std::endl;
        return Eigen::Matrix3d::Identity();
    }

    if (mode_ == 0)
        t = centroid_global - R.transpose() * centroid_base;
    else
        t = threeSidedPositioning(points_global, points_base, matched_gb);
    T.block<2, 2>(0, 0) = R.transpose();
    T.block<2, 1>(0, 2) = t;
    return T;
}

// TODO:?
Eigen::Vector2d PointCloudAlign::threeSidedPositioning(const std::vector<Eigen::Vector2d>& points_global, const std::vector<Eigen::Vector2d>& points_base,
                        const std::vector<int>& matched_gb) {
    // error if the number of points is not equal
    if (points_global.size() != points_base.size()) {
        std::cerr << "The number of points is not equal!" << std::endl;
        return Eigen::Vector2d::Zero();
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(points_global.size() - 1, 2);
    Eigen::VectorXd b = Eigen::MatrixXd::Zero(points_global.size() - 1, 1);
    Eigen::Vector2d point_global_back = points_global.back();
    Eigen::Vector2d point_base_back = points_base[matched_gb.back()];

    for (int i = 0; i < points_global.size() - 1; ++i) {
        Eigen::Vector2d point_global = points_global[i];
        Eigen::Vector2d point_base = points_base[matched_gb[i]];

        A.row(i) = 2 * (point_global - point_global_back).transpose();
        b(i) = point_global.squaredNorm() - point_global_back.squaredNorm() - point_base.squaredNorm() + point_base_back.squaredNorm();
    }

    Eigen::Vector2d X = Eigen::Vector2d::Zero();
    X = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    
    return X;
}
