//
// Created by nepgear on 2024/4/7.
//
#include "lo_v3.h"
#include "pcl/point_types.h"

bool flag = true;
lio this_lio{};

void cloudcallback(const sensor_msgs::PointCloud2::Ptr &msg);

int main(int argc, char **argv) {
    ros::init(argc, argv, "LIO");
    ros::NodeHandle n;
    this_lio.lio_init(n, "/handsfree/imu");
    ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 10000, cloudcallback,
                                         ros::TransportHints().tcpNoDelay());

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

void cloudcallback(const sensor_msgs::PointCloud2::Ptr &msg) {
//    if (!flag) return;
//    for (auto i: msg->fields) {
//        cout << i;
//    }
    //flag = false;
    this_lio.register_pointcloud(msg);
//    if (this_lio.frame_count % 100 == 0) {
//        this_lio.timer_.print_log();
//    }
}