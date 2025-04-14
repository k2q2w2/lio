//
// Created by nepgear on 2024/1/10.
//
#include "gtest/gtest.h"
#include "odom.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include "odom_v2.h"
#include "common.h"
#include "timer.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
void pub_voxel_map(voxelmap &map,ros::Publisher &voxel_pub)
{
    double r = map.resolution;
    visualization_msgs::MarkerArray voxel_plane;
    for(auto &k:map.voxel_map_)
    {
        if(!k.second->is_valid_) continue;
        visualization_msgs::Marker plane;
        plane.header.frame_id="camera_init";
        plane.header.stamp = ros::Time();
        plane.ns = "voxel_planes";
        plane.id = k.second->hash_idx_;
        plane.type = visualization_msgs::Marker::CUBE;
        plane.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Quaternion q;
        double a{0},b{0},c{0};
        a=k.second->f_ver(0);
        b= k.second->f_ver(1);
        c=k.second->f_ver(2);
        double t1 = std::sqrt(a*a+b*b+c*c);
        //if(t1<1e-5) continue;
        a/=t1;b/=t1;c/t1;
        double theta_half = acos(c) / 2;
        double t2 = sqrt(a * a + b * b);
        b = b / t2;
        a = a / t2;
        q.w = cos(theta_half);
        q.x = b * sin(theta_half);
        q.y = -1 * a * sin(theta_half);
        q.z = 0.0;
        plane.pose.orientation = q;
        plane.pose.position.x = floor(k.second->centroid_(0)/r)*r+0.5*r;
        plane.pose.position.y = floor(k.second->centroid_(1)/r)*r+0.5*r;
        plane.pose.position.z = floor(k.second->centroid_(2)/r)*r+0.5*r;
        plane.scale.x = 0.45;
        plane.scale.y = 0.45;
        plane.scale.z = 0.01;
        plane.color.a=1;
        plane.color.r=0;
        plane.color.g=0;
        plane.color.b=255;
        plane.lifetime = ros::Duration();
        voxel_plane.markers.push_back(plane);
    }
    voxel_pub.publish(voxel_plane);
}
//todo: icp done , tbb or omp accelerate and add ros suport;
int main(int argc, char **argv) {
    FLAGS_log_dir = "/home/nepgear/ws_livox/src/lio/log";
    google::InitGoogleLogging(*argv);
    Eigen::setNbThreads(1);
    Eigen::initParallel();
    ros::init(argc, argv, "test_odom");
    ros::NodeHandle n;
    npg::icp test_icp;
    pcl::PointCloud<pcl::PointXYZ> cloud, new_cloud, last_cloud;
    std::string my_pcd = "/home/nepgear/gicp_slam/data/cloud1.pcd";
    pcl::io::loadPCDFile(my_pcd, cloud);
    Eigen::Vector3d axis_rotate(0, 0, -0.3);
    Sophus::SE3d T_star(Sophus::SO3d::exp(axis_rotate), Eigen::Vector3d(-0.3, -0.82, 0.2));
    for (auto i: cloud.points) {
        Eigen::Vector3d p(i.x, i.y, i.z);
        if (p.norm() <= 100) {
            Eigen::Vector3d p2 = T_star * p;
            pcl::PointXYZ p3;
            p3.x = p2(0);
            p3.y = p2(1);
            p3.z = p2(2);
            new_cloud.points.push_back(p3);
            last_cloud.points.push_back(i);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_ptr(new pcl::PointCloud<pcl::PointXYZ>), last_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
    *new_ptr = new_cloud;
    *last_ptr = last_cloud;
//    *new_ptr = *test_icp.icp_downsample(new_ptr);
//    *last_ptr = *test_icp.icp_downsample(last_ptr);
//    test_icp.deskewed_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//    test_icp.last_scan_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//    test_icp.registerlaserdata(last_ptr);
//
//    test_icp.registerlaserdata(new_ptr);
    LO_gicp test_icp0{};
  // for(int i=0;i<14;i++)
    test_icp0.LO(last_ptr);
    test_icp0.lidar_pose_ =  Sophus::SE3d (Sophus::SO3d::exp(-axis_rotate), Eigen::Vector3d(0.4,0.8,-0.0));
   test_icp0.LO(new_ptr);
   // test_icp0.LO(last_ptr);
    std::cout << test_icp0.lidar_pose_.translation().x() << std::endl;
    std::cout << test_icp0.lidar_pose_.translation().y() << std::endl;
    std::cout << test_icp0.lidar_pose_.translation().z() << std::endl;

    std::cout << test_icp0.lidar_pose_.angleX() << std::endl;
    std::cout << test_icp0.lidar_pose_.angleY() << std::endl;
    std::cout << test_icp0.lidar_pose_.angleZ() << std::endl;
    ros::Publisher voxel_map_pub = n.advertise<visualization_msgs::MarkerArray>("/planes",10000);
    ros::Publisher pcl_send = n.advertise<sensor_msgs::PointCloud2>("vel_points",10);
    //std::cout << test_icp0.icp_point_count << std::endl;
    //std::cout << test_icp.lidar_pose_.inverse(). translation() << std::endl;
    //std::cout << test_icp.ll_cost << std::endl;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*test_icp0.deskewed_scan,output);
    ros::AsyncSpinner spinner(0);
    spinner.start();ros::Rate cr(1);
    while(ros::ok())
    {
        cr.sleep();
        pub_voxel_map(test_icp0.voxel_map_,voxel_map_pub);
        output.header.frame_id="camera_init";
        pcl_send.publish(output);
    }

}