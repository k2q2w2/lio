//
// Created by nepgear on 2024/2/29.
//

#ifndef LIO_COMMON_H
#define LIO_COMMON_H

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
#include "sophus/so3.hpp"
#include "nanoflann.hpp"

//#include "tqdm/tqdm.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include "chrono"
#include "unordered_map"
#include "glog/logging.h"
#include "fstream"
#include "sstream"
//#include "Eigen/StdDeque"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "Eigen/Core"

#endif //LIO_COMMON_H
