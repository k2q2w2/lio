//
// Created by nepgear on 2024/4/7.
//
#include "imu.h"

void imu::imu_callback(const sensor_msgs::Imu::ConstPtr &msg_ptr) {
    sensor_msgs::Imu new_msgs=*msg_ptr;
    current_timestamp_ = new_msgs.header.stamp.toSec();
    std::lock_guard<std::mutex> lock(imu_lock_);
    if(current_timestamp_<last_timestamp_)
    {
        std::cout<<"loop_back!"<<std::endl;
        imu_buff_.clear();
        return;
    }
    last_timestamp_ = current_timestamp_;
    sensor_msgs::Imu filter_msgs=*msg_ptr;
    double ema_beta = 0.8,ema_alpha = 1-ema_beta;
    filter_msgs.linear_acceleration.x = ema_beta*new_msgs.linear_acceleration.x+ema_alpha
            *last_imu_data_.linear_acceleration.x;
    filter_msgs.linear_acceleration.y = ema_beta*new_msgs.linear_acceleration.y+ema_alpha
            *last_imu_data_.linear_acceleration.y;
    filter_msgs.linear_acceleration.z = ema_beta*new_msgs.linear_acceleration.z+ema_alpha
            *last_imu_data_.linear_acceleration.z;
    last_imu_data_ = new_msgs;
    imu_buff_.push_back(filter_msgs);
}
imu::imu(ros::NodeHandle &n, std::string &imu_topic_name) {
    imu_sub_ = n.subscribe<sensor_msgs::Imu>(imu_topic_name,1000,&imu::imu_callback,this);
}