//
// Created by nepgear on 2024/4/7.
//
#pragma once

#include "common.h"

class imu {

public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ros::Subscriber imu_sub_{};
    std::deque<sensor_msgs::Imu> imu_buff_{};
    double last_timestamp_{}, current_timestamp_{};
    std::mutex imu_lock_;
    sensor_msgs::Imu last_imu_data_{};
    bool loosely_couple_flag_ = false;
    sensor_msgs::Imu last_msg_;

    imu(ros::NodeHandle &n, std::string &imu_topic_name);
    void clear();//TODO:retain last imu msg;
    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg_ptr);
    //void intergrate();
};