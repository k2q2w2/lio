//
// Created by nepgear on 2023/12/12.
//
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
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
#include "sophus/se3.hpp"

int count0 = 0;

void pointcloudcallback_test(const sensor_msgs::PointCloud2ConstPtr &pc) {
    if (count0 > 50) return;
    using namespace std;
    cout << count0 << endl;
    for (auto &field: pc->fields) {
        cout << field.name << endl;
    }
    count0++;
}

int main(int argc, const char **argv) {
    Eigen::Vector3d v(1, 1, 0.5);
    Sophus::SO3d l();
    ros::init(argc, const_cast<char **>(argv), "test_pc_fields");
    ros::NodeHandle n;
    ros::Subscriber pc_sub = n.subscribe("/livox/lidar", 10000, pointcloudcallback_test,
                                         ros::TransportHints().tcpNoDelay());
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}