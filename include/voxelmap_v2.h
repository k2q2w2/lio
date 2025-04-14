//
// Created by nepgear on 2024/4/16.
//

#pragma once

#include "common.h"
#include "execution"

namespace npg_v2 {

    class grid {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        grid() = default;

        size_t point_count_ = 0;
        size_t hash_idx_ = 0;
        int first_index = 0;
        Eigen::Matrix3d cov_sum_{};
        Eigen::Vector3d centroid_{0.0f, 0.0f, 0.0f};
        Eigen::Matrix<double, 3, 3> conv_ = Eigen::Matrix3d::Zero();
        Eigen::Vector3d points_sum_{0.0f, 0.0f, 0.0f};
        std::vector<Eigen::Vector3d>
                points_array_{};//TODO:使用双端队列，头部匹配点，尾部匹配回环点
        bool is_valid_ = false;
        Eigen::Vector3d f_ver = Eigen::Vector3d::Zero();
    };

    class knn_point {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3d point_{};
        double point_dist_{};

        knn_point(Eigen::Vector3d p, double point_dist) : point_dist_(point_dist), point_(p) {};

        inline bool operator<(const knn_point &p2) {
            return point_dist_ < p2.point_dist_;
        };
    };

    class Voxelmap {
    public:
        using M3d = Eigen::Matrix<double, 3, 3>;
        using V3d = Eigen::Vector3d;

        int num_thread_ = 8;
        int points_with_covs = 0;
        bool nearby_ = true;
        size_t HASH_P = 116101;
        size_t MAX_N = 10000000000;
        double resolution = 1.0f;
        size_t capacity_ = 2000000;
        size_t max_gird_point_ = 50;

        std::unordered_map<size_t, std::shared_ptr<grid>> voxel_map_{};

        void AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, bool cov_flag);

        bool KNN_search(Eigen::Vector3d &point, size_t n, double range,
                        std::vector<Eigen::Vector3d> &result);

        void caculate_conc(std::shared_ptr<grid> &this_grid);

        Voxelmap(bool near7_flag, double res);

        std::vector<Eigen::Matrix<double, 3, 1>> delta_near_;

        size_t caculate_hash_index(Eigen::Vector3d &&p);

        size_t caculate_hash_index(Eigen::Vector3d &p);


    };


}