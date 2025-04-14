//
// Created by nepgear on 2024/1/16.
//
#include "LO.h"

npg::icp LO_icp;

void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    //msg2pc
    auto start = std::chrono::system_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), clouds(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_ptr);
    //pc2downsample
    clouds = LO_icp.icp_downsample(cloud_ptr);
    LO_icp.registerlaserdata(clouds);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "total time:" << elapsed_seconds.count() << std::endl;
    //GN_step
    //build kdtree
    return;
}

void livox_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>), clouds(
            new pcl::PointCloud<pcl::PointXYZ>);
    for (auto i: msg->points) {
        pcl_cloud->points.push_back(pcl::PointXYZ(i.x, i.y, i.z));
    }
    auto start = std::chrono::system_clock::now();
    clouds = LO_icp.icp_downsample(pcl_cloud);
    LO_icp.registerlaserdata(clouds);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "location is" << LO_icp.lidar_pose_.inverse().translation() << "\t";
    std::cout << "total time:" << elapsed_seconds.count() << std::endl;
    //GN_step
    //build kdtree
    return;
}

void figure_callback(const std_msgs::Bool::ConstPtr &msgs) {
    if (msgs->data) {
        printf("get type true");
        std::ofstream ss;
        ss.open("/home/nepgear/ws_livox/src/lio/log/10.txt", std::ios::ate);
        std::cout << LO_icp.pos_res_[0].size() << " " << LO_icp.pos_res_.size() << std::endl;
        for (int i = 0; i < LO_icp.pos_res_.size(); i++) {
            for (int j = 0; j < LO_icp.pos_res_[0].size(); j++) {
                ss << LO_icp.pos_res_[i][j];
                if (j != LO_icp.pos_res_[0].size() - 1) {
                    ss << " ";
                }
            }
            ss << std::endl;
        }
        ss << std::flush;
        ss.close();
    } else {
        printf("get type false");
    }

}

int main(int argc, char **argv) {
    FLAGS_log_dir = "/home/nepgear/ws_livox/src/lio/log";
    google::InitGoogleLogging(*argv);
    Eigen::setNbThreads(16);
    Eigen::initParallel();
    ros::init(argc, argv, "LO");
    ros::NodeHandle n;
    ros::param::get("max_min_icp_distance", LO_icp.max_min_icp_distance_);
    ros::param::get("max_iterations", LO_icp.max_iterations_);
    ros::param::get("leaf_size", LO_icp.leaf_size_);
    ros::param::get("factor", LO_icp.factor_);
    LO_icp.deskewed_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    LO_icp.last_scan_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 10000, cloudcallback,
                                         ros::TransportHints().tcpNoDelay());
    ros::Subscriber livox_sub = n.subscribe("/livox/lidar", 10000, livox_callback,
                                            ros::TransportHints().tcpNoDelay());
    ros::Subscriber show_figure = n.subscribe("/lidar_figure", 1, figure_callback);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}