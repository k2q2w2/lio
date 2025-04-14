//
// Created by nepgear on 2024/2/29.
//

#include "lo_v2.h"
#include "odom_v2.h"
#include "std_msgs/Bool.h"
#include "matplotlibcpp.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

LO_gicp this_gicp{};
ros::Publisher pc_pub, originpc_pub;

void pub_voxel_map(voxelmap &map, ros::Publisher &voxel_pub);

void pub_pointcloud();

void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    //msg2pc
    originpc_pub.publish(*msg);
    auto start = std::chrono::system_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), clouds(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_ptr);
    auto flags = this_gicp.LO(cloud_ptr);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "total time:" << elapsed_seconds.count() << std::endl;
    return;
}

void figure_callback(const std_msgs::Bool::ConstPtr &msgs) {
    if (msgs->data) {
        printf("get type true");
        std::ofstream ss;
        ss.open("/home/nepgear/ws_livox/src/lio/log/10.txt", std::ios::ate);
        std::cout << this_gicp.pos_res_[0].size() << " " << this_gicp.pos_res_.size() << std::endl;
        for (int i = 0; i < this_gicp.pos_res_.size(); i++) {
            for (int j = 0; j < this_gicp.pos_res_[0].size(); j++) {
                ss << this_gicp.pos_res_[i][j];
                if (j != this_gicp.pos_res_[0].size() - 1) {
                    ss << " ";
                }
            }
            ss << std::endl;
        }
        ss << std::flush;
        ss.close();
        this_gicp.timer_.print_log();
    } else {
        printf("get type false");
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "LO");
    ros::NodeHandle n;
    ros::Subscriber pc_sub = n.subscribe("/velodyne_points", 10000, cloudcallback,
                                         ros::TransportHints().tcpNoDelay());
    ros::Subscriber show_figure = n.subscribe("/lidar_figure", 1, figure_callback);
    ros::Publisher voxel_map_pub = n.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
    pc_pub = n.advertise<sensor_msgs::PointCloud2>("downsample_pc", 10);
    originpc_pub = n.advertise<sensor_msgs::PointCloud2>("origin_pc", 10);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Rate r(1);
    while (1) {
        r.sleep();
        pub_voxel_map(this_gicp.voxel_map_, voxel_map_pub);
        //pub_pointcloud();
    }
    ros::waitForShutdown();
    return 0;
}

void pub_voxel_map(voxelmap &map, ros::Publisher &voxel_pub) {
    visualization_msgs::MarkerArray voxel_plane;
    for (auto &k: map.voxel_map_) {
        visualization_msgs::Marker plane;
        plane.header.frame_id = "camera_init";
        plane.header.stamp = ros::Time();
        plane.ns = "voxel_planes";
        plane.id = k.second->hash_idx_;
        plane.type = visualization_msgs::Marker::CUBE;
        plane.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Quaternion q;
        double a{0}, b{0}, c{0};
        a = k.second->f_ver(0);
        b = k.second->f_ver(1);
        c = k.second->f_ver(2);
        double t1 = std::sqrt(a * a + b * b + c * c);
        a /= t1;
        b /= t1;
        c / t1;
        double theta_half = acos(c) / 2;
        double t2 = sqrt(a * a + b * b);
        b = b / t2;
        a = a / t2;
        q.w = cos(theta_half);
        q.x = b * sin(theta_half);
        q.y = -1 * a * sin(theta_half);
        q.z = 0.0;
        plane.pose.orientation = q;
        plane.pose.position.x = floor(k.second->centroid_(0)) + 0.5;
        plane.pose.position.y = floor(k.second->centroid_(1)) + 0.5;
        plane.pose.position.z = floor(k.second->centroid_(2)) + 0.5;
        plane.scale.x = 0.45;
        plane.scale.y = 0.45;
        plane.scale.z = 0.01;
        plane.color.a = 50 / k.second->point_count_;
        plane.color.r = 255;
        plane.color.g = 255;
        plane.color.b = 255;
        plane.lifetime = ros::Duration();
        voxel_plane.markers.push_back(plane);
    }
    voxel_pub.publish(voxel_plane);
}

void pub_pointcloud() {
    sensor_msgs::PointCloud2 this_scan;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pub_scan;
    pub_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &i: this_gicp.voxel_map_.voxel_map_) {
        pcl::PointXYZ new_point = pcl::PointXYZ(i.second->centroid_(0), i.second->centroid_(1), i.second->centroid_(2));
        pub_scan->push_back(new_point);
    }
    pcl::toROSMsg(*pub_scan, this_scan);
    this_scan.header.frame_id = "camera_init";
    this_scan.header.stamp = ros::Time::now();
    pc_pub.publish(this_scan);
}