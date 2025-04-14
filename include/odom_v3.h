//
// Created by nepgear on 2024/4/7.
//

#pragma once

#include "common.h"
#include "sensor_msgs/PointCloud2.h"
#include "timer.h"
#include "imu.h"
#include "voxelmap_v2.h"

#define TEST_UNDISTORTED 1

class lio {
public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Eigen::Matrix<double, 3, 3> Matrix33;
    typedef Eigen::Vector3d V3d;
    enum lidar_type {
        velodyne, livox
    };

    void register_pointcloud(const sensor_msgs::PointCloud2::Ptr &msg);

    void preprocess(const sensor_msgs::PointCloud2::Ptr &msg,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out);

    void field_check(const sensor_msgs::PointCloud2::Ptr &msg);

    void lio_init(ros::NodeHandle &n, std::string imutopicname);

    void radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

    void scan2cov(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &output, double res,
                  bool cov_flag, bool near7_flag);

    void nominate_update(double current_time, double last_time);

    void linearize(Eigen::Matrix<double, 15, 15> &H, Eigen::Matrix<double, 15, 1> &g,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in);

    void imu_linearize(Eigen::Matrix<double, 15, 15> &H, Eigen::Matrix<double, 15, 1> &g);

    void imu_AHRS(std::shared_ptr<imu> &imu);

    Eigen::Matrix3d jr_inv(V3d e_lie);

    double cur_time_{}, last_time_{0};
    /*名义状态变量*/
    V3d nominate_p_ = V3d::Zero(), nominate_v_ = V3d::Zero(), ba = V3d::Zero(), bg = V3d::Zero();
    Matrix33 nominate_R_ = Matrix33::Zero();
    /*F,CONV*/
    std::shared_ptr<npg_v2::Voxelmap> voxelmap_;
    Eigen::Matrix<double, 18, 18> F_, CONV_, Q_;
    std::vector<Matrix33> point_with_cov_;
    std::mutex lio_guard_;
    Timer timer_{};
    size_t frame_count = 0;
    lidar_type lidar_type_{velodyne};
    bool field_check_flag_ = true, has_time = false;
    struct offsetXYZT {
        std::uint32_t offset_x{0};
        std::uint32_t offset_y{4};
        std::uint32_t offset_z{8};
        std::uint32_t offset_time{0};
    };
    Sophus::SE3d cur_pose_{}, key_pose_{}, last_pose_{};
    V3d last_vel, cur_vel;
    struct offsetXYZT point_field_offset_{};
    std::shared_ptr<imu> imu_;//TODO:need to be initialize
    V3d imu_angualr_bias{}, imu_accel_bias{};
    double filter_size = 1.0f;
    int feat_num_{0};
    int max_iteration_{10};
    bool imu_AHRS_flag_ = false;
    ros::Publisher deskew_scan_pub_, body_scan_;//TODO:RAII modify?
};