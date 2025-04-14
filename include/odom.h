//
// Created by nepgear on 2023/12/13.
//

#ifndef LIO_ODOM_H
#define LIO_ODOM_H
//
// Created by nepgear on 2023/12/13.
//
#include "ros/ros.h"
#include <iostream>
#include "pcl_ros/point_cloud.h"
#include <mutex>
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include "nanoflann.hpp"
#include <livox_sdk.h>
#include "livox_lidar_api.h"
#include "livox_ros_driver/CustomMsg.h"
#include "livox_ros_driver/CustomPoint.h"
#include "queue"
#include "sophus/se3.hpp"
#include "nanoflann.hpp"
//#include "tqdm/tqdm.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "chrono"

namespace npg {


    class IMU_data {
    public:
        double time;
        Eigen::Vector3d gyro, accel;
    };


    class KdtreeAdaptor {
    public:
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_;

        inline size_t kdtree_get_point_count() const {
            return scan_->size();

        }

        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            if (dim == 0) {
                return scan_->at(idx).x;
            }
            if (dim == 1) {
                return scan_->at(idx).y;
            }
            if (dim == 2) {
                return scan_->at(idx).z;
            }
        }

        template<class BBOX>
        bool kdtree_get_bbox(BBOX &) const {
            return false;
        }
    };

    class icp {
        //似然场法?
    public:
        bool debug_mode_ = true;
        double max_min_icp_distance_ = 0.5f, ll_cost;
        double leaf_size_ = 1.0f;
        int icp_point_count = 0;
        double factor_ = 1.0f;
        std::vector<std::vector<double>> pos_res_;
        std::mutex mtx;
        std::vector<IMU_data> imu_buffer;
        Eigen::Matrix<double, 3, 3> R_scans_;
        Eigen::Matrix<double, 3, 1> P_scans_;
        Eigen::Matrix<double, 3, 1> V_scans_;
        Eigen::Matrix<double, 3, 3> R_imu_;
        Eigen::Matrix<double, 3, 1> P_imu_;
        Sophus::SE3d lidar_pose_,last_poses_;
        int max_iterations_ = 10;
        int frame_count = 0;
        int keyframe_count = 1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr deskewed_scan, last_scan_;
        int imu_index = 0;
        double last_lidar_time_ = 0;
        KdtreeAdaptor this_scan_in_kdtree;
        typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor
                <double, KdtreeAdaptor>, KdtreeAdaptor, 3> my_kd_tree;
        std::shared_ptr<my_kd_tree> index_;
        bool first_scan_valid_ = false;
        Eigen::Vector3d delta_x_{};
        //void init();
        //void registerlaserdata(const livox_ros_driver::CustomMsg::ConstPtr &msg);

        void registerlaserdata(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

        Sophus::SE3d GaussianNewton();

        pcl::PointCloud<pcl::PointXYZ>::Ptr icp_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg);

        void imu_callback();

        //void deskewedPoints(pcl::PointCloud<livox_ros_driver::CustomPoint>::Ptr &pc, double cur_time);
    };
}

#endif //LIO_ODOM_H
