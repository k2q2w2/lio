//
// Created by nepgear on 2024/9/29.
//

#pragma once

#include "common.h"
#include "sensor_msgs/PointCloud2.h"
#include "timer.h"
#include "imu.h"
#include "voxelmap_v2.h"
#include "execution"
namespace gear_lio{
    class lio_config {
        std::string imu_topic_name;
        std::string lidar_topic_name;
        ros::NodeHandle n;
    };
    struct offsetXYZT {
        std::uint32_t offset_x{0};
        std::uint32_t offset_y{4};
        std::uint32_t offset_z{8};
        std::uint32_t offset_time{0};
    };

    class lio {
    public:
        enum lidar_type {
            velodyne, livox
        };
        typedef Eigen::Matrix3d Mat33;
        typedef Eigen::Vector3d V3d;
        //地图
        std::shared_ptr<npg_v2::Voxelmap> voxelmap_;
        std::vector<Mat33> point_with_cov_{};
        //thread_locker
        std::mutex lio_guard_;
        //计时器
        Timer timer_{};
        //count tools
        size_t frame_count = 0;
        size_t feat_nums_ = 0;
        //p,v,R
        Mat33 nominate_R_ = Mat33::Zero();
        V3d nominate_V_ = V3d::Zero(), nominate_P_ = V3d::Zero();
        Sophus::SE3d cur_pose_{}, key_pose_{}, last_pose_{};
        std::vector<Sophus::SE3d> key_poses_container_{};
        V3d last_vel_{}, key_vel_{}, cur_vel_{};
        //flag;
        bool imu_AHRS_flag_ = false;
        bool field_check_flag_ = false, has_time = false;
        //icp config
        int max_iteration_{10};
        double filter_size = 1.0f;
        //imu_config
        std::shared_ptr<imu> &imu_;//TODO:init in lio_init
        V3d imu_angualr_bias{}, imu_accel_bias{};
        //ros interface
        ros::Publisher deskew_scan_pub_, body_scan_;
        ros::Subscriber imu_sub_, lidar_sub;
        //time
        double cur_time=0,last_time=0;
        //cov
        std::vector<Mat33> covariance_;
        //pc_field
        struct offsetXYZT point_field_offset_{};
        lidar_type lidar_type_{velodyne};
        //filter parameters
        bool radius_filter_flag_ {true};
        float maximum_radius_2{9.0f};
        /*工作流程：
         * TODO：
         * 1.初始化订阅话题+外参设置（imu+lidar）-> 2.imu零漂估计
         *                             loop:->1. 点云预处理(voxel_downsample)+radius_filter
         *                                  ->2. 地图初始化
         *                                  ->3. nominate_update
         *                                  ->4. gicp线性化
         *                                  ->5. imu线性化
         *                                  ->6. 优化
         *                                  ->7. 关键帧更新
         * */
        void pipeline(const sensor_msgs::PointCloud2::Ptr &msg);

        void lio_init(lio_config &cfg);
        void imu_callback();
        void field_check(const sensor_msgs::PointCloud2::Ptr &msg);
        void imu_AHRS(std::shared_ptr<imu> &imu);
        void preprocess(const sensor_msgs::PointCloud2::Ptr &msg, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_out);
        void distort(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud);
        void filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
        void compute_covariance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud,std::vector<Mat33> cov);
        void predict();
        void linearize();
        void optimization();
        void Keyframe_update();
        template<typename T>
        float compute_radius(T &p) {
            return p.x*p.x+p.y*p.y+p.z*p.z;
        }
    };
}