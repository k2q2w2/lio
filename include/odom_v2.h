//
// Created by nepgear on 2024/3/5.
//

#pragma once

#include "timer.h"
#include "common.h"
#include "voxelmap.h"

class LO_gicp {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void GN_optimization();

    bool LO(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

    void scan2scanCov(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

    void scan2scanCov(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, double res);

    size_t caculate_hash_index(Eigen::Vector3d &p);

    LO_gicp();

    void Inradius(pcl::PointCloud<pcl::PointXYZ>::Ptr &Input,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr &Ouput);

    void CauchyLossFunction(const double e,
                            const double delta,
                            Eigen::Vector3d &rho);

    std::mutex mtx;
    size_t HASH_P = 116101;
    size_t MAX_N = 10000000000;
    double resolution = 1.0f;//注意voxelmap resolution
    int frame_count_ = 0;
    std::vector<std::vector<double>> pos_res_;
    Eigen::Matrix<double, 3, 3> R_scans_;
    Eigen::Matrix<double, 3, 1> P_scans_;
    Eigen::Matrix<double, 3, 1> V_scans_;
    Eigen::Matrix<double, 3, 3> R_imu_;
    Eigen::Matrix<double, 3, 1> P_imu_;
    Sophus::SE3d lidar_pose_{}, keyframe_pose_{};
    int max_iterations_ = 10;
    double leaf_size_ = 1.0f;
    int keyframe_count = 0;
    int icp_point_count = 0, effect_feat_num = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr deskewed_scan;//TODO：需要初始化
    voxelmap voxel_map_{};
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> point_cov_{};
    Timer timer_{};
    int num_threads_ = 8;
};