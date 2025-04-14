//
// Created by nepgear on 2024/4/7.
//
#include "odom_v3.h"
#include "timer.h"
#include <exception>

//TODO：1.tf 转换
//2.检查IMU

void lio::register_pointcloud(const sensor_msgs::PointCloud2::Ptr &msg) {
    //lock;
    std::lock_guard<std::mutex> lock(lio_guard_);
    //step0:preprocess
    cur_time_ = msg->header.stamp.toSec();
    if (last_time_ == 0) {
        last_time_ = cur_time_;
        return;
    }

    //添加初始化flag，以及imu初始化
    if (imu_->imu_buff_.size() > 20 && !imu_AHRS_flag_) {
        imu_AHRS(imu_);
        imu_AHRS_flag_ = true;
#ifdef TEST_IMU_AHRS
        std::cout<<frame_count<<std::endl;
#endif
    }

    frame_count++;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_undistorted(new pcl::PointCloud<pcl::PointXYZ>), cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>), cloud_ptr_cov(
            new pcl::PointCloud<pcl::PointXYZ>);
    //step1:undistorted pointcloud
    if (1) {
        timer_.timer("preprocess", [&, this]() { preprocess(msg, cloud_undistorted); });
        timer_.timer("radius", [&, this]() { radius_filter(cloud_undistorted, cloud_ptr); });
        if (frame_count < 10) {
            timer_.timer("init_map", [&, this]() {
                voxelmap_->AddPointCloud(cloud_ptr, true);
            });
            return;
        }
    } else {
        return;
    }
#ifdef TEST_UNDISTORTED
    sensor_msgs::PointCloud2 this_scan;
    pcl::toROSMsg(*cloud_undistorted, this_scan);
    this_scan.header.frame_id = "velodyne";
    this_scan.header.stamp = ros::Time::now();
    deskew_scan_pub_.publish(this_scan);
#endif
    std::cout << frame_count << std::endl;
    //step2:downsample pointcloud
    timer_.timer("downsample&cov", [&, this]() { scan2cov(cloud_ptr, cloud_ptr_cov, filter_size, true, false); });

    //step3:predict&update nominate state;
    timer_.timer("predict", [&, this]() { nominate_update(cur_time_, last_time_); });
    //step4:optimization
    timer_.timer("optimization", [&, this]() {
        Eigen::Matrix<double, 15, 15> H = Eigen::Matrix<double, 15, 15>::Zero();
        Eigen::Matrix<double, 15, 1> g = Eigen::Matrix<double, 15, 1>::Zero();
        Eigen::Matrix<double, 15, 1> delta_x;
        for (int it_i = 0; it_i < max_iteration_; it_i++) {
            linearize(H, g, cloud_ptr_cov);
#ifndef ICP_CHECK_FLAG
            Eigen::Matrix<double, 15, 1> delta_x_v = -H.inverse() * g;
            std::cout << "trans:" << delta_x_v.block<3, 1>(0, 0);
            std::cout << "ori" << delta_x_v.block<3, 1>(6, 0) << std::endl;
#endif
            //imu线性化
            //imu_linearize(H, g);
            delta_x = -H.inverse() * g;
            if (delta_x.lpNorm<Eigen::Infinity>() < 1e-3 || delta_x.hasNaN()) {
                break;
            }
            //更新当前状态：
            //last_pose_ = cur_pose_;
            cur_pose_ *= Sophus::SE3d(Sophus::SO3d::exp(delta_x.block<3, 1>(6, 0)), delta_x.head<3>());
            cur_vel += delta_x.block<3, 1>(3, 0);
            ba += delta_x.block<3, 1>(9, 0);
            bg += delta_x.block<3, 1>(12, 0);
            //TODO:验证结果是否正确,跟踪更新后的数值
        }
        Eigen::Matrix<double, 15, 15> L = Eigen::Matrix<double, 15, 15>::Identity();
        L.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() -
                              0.5 * Sophus::SO3d::hat(delta_x.block<3, 1>(6, 0));
        //step4.5:reset filter
        //更新协方差并重置滤波器
        CONV_.block<15, 15>(0, 0) = L * H * L;
    });
    //step5:keyframe update & update voxel map

    timer_.timer("keyframeUpdate", [&, this]() {
        Sophus::SE3d delta = key_pose_.inverse() * cur_pose_;
        if (delta.translation().norm() > 0.5 || delta.so3().log().norm() > 0.18) {
            voxelmap_->AddPointCloud(cloud_ptr_cov, true);
            key_pose_ = cur_pose_;
        }
        last_pose_ = cur_pose_;

    });
    //更新last_pose



    last_time_ = cur_time_;
}

void lio::preprocess(const sensor_msgs::PointCloud2::Ptr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) {
    //1.fields_check
    if (field_check_flag_) {
        field_check(msg);
        field_check_flag_ = false;
    }
    size_t point_cloud_size = msg->height * msg->width;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr origin_pointcloud(new pcl::PointCloud<pcl::PointXYZINormal>),
            distorted_pointcloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    origin_pointcloud->reserve(point_cloud_size);
    //TODO:add if has no time

    if (has_time) {
        size_t point_step = msg->point_step;
        for (size_t i = 0; i < point_cloud_size; i++) {
            pcl::PointXYZINormal new_point;
            new_point.x = *(float *) (&msg->data[0] + point_field_offset_.offset_x + point_step * i);
            new_point.y = *(float *) (&msg->data[0] + point_field_offset_.offset_y + point_step * i);
            new_point.z = *(float *) (&msg->data[0] + point_field_offset_.offset_z + point_step * i);
            if (lidar_type_ == velodyne) {
                new_point.curvature =
                        *(float *) (&msg->data[0] + point_field_offset_.offset_time + point_step * i);//unit:s
            }
            origin_pointcloud->push_back(new_point);
        }
    }

    std::function<bool(pcl::PointXYZINormal &, pcl::PointXYZINormal &)> point_cmp;
    distorted_pointcloud->resize(point_cloud_size);
    if (lidar_type_ == velodyne) {
        point_cmp = [](pcl::PointXYZINormal &p1, pcl::PointXYZINormal &p2) {
            return p1.curvature < p2.curvature;
        };
    }
    //sorted by time

    std::partial_sort_copy(origin_pointcloud->begin() + 1, origin_pointcloud->end(),
                           distorted_pointcloud->begin() + 1, distorted_pointcloud->end(), point_cmp);

    //integrate imu measurement
    double ros_bag_time =
            msg->header.stamp.toSec() + distorted_pointcloud->at(0).curvature;//all in to unit:/s start_time;
    double imu_first_time = imu_->imu_buff_[0].header.stamp.toSec();
    double lidar_end_time = msg->header.stamp.toSec() + distorted_pointcloud->at(point_cloud_size - 1).curvature;
    double baseline_time = msg->header.stamp.toSec();
    //deprecate unuse imu time;

    Matrix33 R = Matrix33::Identity();
    V3d T = V3d::Zero();
    cloud_out->resize(point_cloud_size);
    size_t index = 0;
    double last_imu_time = ros_bag_time;
    //if (imu_->imu_buff_.empty()) return;

    for (auto &i: imu_->imu_buff_) {
        double imu_time = i.header.stamp.toSec();
        if (imu_time <= ros_bag_time) {//起始时间后
            continue;
        }
        V3d angualr = V3d(i.angular_velocity.x, i.angular_velocity.y, i.angular_velocity.z);
        V3d linear = V3d(i.linear_acceleration.x, i.linear_acceleration.y, i.linear_acceleration.z);
        while (index < point_cloud_size && distorted_pointcloud->at(index).curvature + baseline_time <= imu_time
                ) {

            double dt = distorted_pointcloud->at(index).curvature + baseline_time - last_imu_time;
            Matrix33 R_t = R * Sophus::SO3d::exp(dt * (angualr - imu_angualr_bias)).matrix();
            V3d T_t = T + 0.5 * (linear - imu_accel_bias) * dt * dt;

            V3d np = R_t * V3d(distorted_pointcloud->at(index).x, distorted_pointcloud->at(index).y,
                               distorted_pointcloud->at(index).z) + T_t;
            pcl::PointXYZ n_p(np.x(), np.y(), np.z());
            cloud_out->push_back(n_p);
            index++;
        }
        if (index >= point_cloud_size) break;
        //update T,R
        double per_time = imu_time - last_imu_time;
        T = T + 0.5 * (linear - imu_accel_bias) * per_time * per_time;
        R = R * Sophus::SO3d::exp(per_time * (angualr - imu_angualr_bias)).matrix();
        last_imu_time = imu_time;
    }


}


void lio::field_check(const sensor_msgs::PointCloud2::Ptr &msg) {
    for (auto &i: msg->fields) {
        if (i.name == "x") {
            point_field_offset_.offset_x = i.offset;
        }
        if (i.name == "y") {
            point_field_offset_.offset_y = i.offset;
        }
        if (i.name == "z") {
            point_field_offset_.offset_z = i.offset;
        }
        //velodyne
        if (i.name == "time") {
            point_field_offset_.offset_time = i.offset;
            has_time = true;
        }
    }
}

void lio::lio_init(ros::NodeHandle &n, std::string imutopicname) {
    imu_ = std::make_shared<imu>(n, imutopicname);
    deskew_scan_pub_ = n.advertise<sensor_msgs::PointCloud2>("undistort_pointcloud", 10);
    body_scan_ = n.advertise<sensor_msgs::PointCloud2>("body_scan", 10);
    Q_.block<9, 9>(0, 0) = Eigen::Matrix<double, 9, 9>::Identity() * 1e-2;
    voxelmap_ = std::make_shared<npg_v2::Voxelmap>(true, 1.0f);
    CONV_ = Eigen::Matrix<double, 18, 18>::Identity();
    //todo:add static init bias
}

void lio::radius_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &output) {
    for (auto i: *msg) {
        Eigen::Vector3f point = Eigen::Vector3f(i.x, i.y, i.z);
        if (point.norm() <= 150.0f && point.norm() >= 0.5f) {
            pcl::PointXYZ np(point.x(), point.y(), point.z());
            output->push_back(np);
        }
    }

}

void lio::scan2cov(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &output, double res,
                   bool cov_flag, bool near7_flag) {
    npg_v2::Voxelmap *vm = new npg_v2::Voxelmap(near7_flag, res);
    vm->AddPointCloud(msg, cov_flag);
    for (const auto &i: vm->voxel_map_) {
        output->push_back(pcl::PointXYZ(i.second->centroid_.x(), i.second->centroid_.y(), i.second->centroid_.z()));
        if (cov_flag) {
            point_with_cov_.emplace_back(i.second->conv_);
        }
    }
}

void lio::nominate_update(double current_time, double last_time) {
    while (!imu_->imu_buff_.empty() && imu_->imu_buff_.front().header.stamp.toSec() < current_time) {
        //tackling data:
        sensor_msgs::Imu topMeasurement = imu_->imu_buff_.front();
        double dt = topMeasurement.header.stamp.toSec() - last_time;
        V3d w = V3d(topMeasurement.angular_velocity.x, topMeasurement.angular_velocity.y,
                    topMeasurement.angular_velocity.z);
        V3d a = V3d(topMeasurement.linear_acceleration.x, topMeasurement.linear_acceleration.y,
                    topMeasurement.linear_acceleration.z);
        cur_pose_.translation() += cur_vel * dt + 0.5 * (cur_pose_.rotationMatrix() * (a - ba)) * dt * dt;
        cur_vel += cur_pose_.rotationMatrix() * (a - ba) * dt;
        cur_pose_.so3() *= Sophus::SO3d::exp((w - bg) * dt);
        F_.setIdentity();//p,v,R
        F_.block<3, 3>(0, 3) = Matrix33::Identity() * dt;
        F_.block<3, 3>(3, 6) = -cur_pose_.rotationMatrix() * Sophus::SO3d::hat(a - ba) * dt;
        F_.block<3, 3>(3, 12) = -cur_pose_.rotationMatrix() * dt;
        F_.block<3, 3>(3, 15) = -Matrix33::Identity() * dt;
        F_.block<3, 3>(6, 6) = Sophus::SO3d::exp(-(w - bg) * dt).matrix();
        F_.block<3, 3>(6, 9) = -Matrix33::Identity() * dt;
        CONV_ = F_ * CONV_.eval() * F_.transpose() + Q_;
        imu_->imu_buff_.pop_front();
    }
}

void lio::linearize(Eigen::Matrix<double, 15, 15> &H, Eigen::Matrix<double, 15, 1> &g,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in) {
    feat_num_ = 0;
    size_t index = cloud_in->size();
    for (int i = 0; i < index; ++i) {
        V3d this_point = V3d(cloud_in->at(i).x, cloud_in->at(i).y, cloud_in->at(i).z);
        Matrix33 pa_cov = point_with_cov_[i];
        V3d p_a = cur_pose_ * this_point;
        Eigen::Matrix<double, 3, 6> J;
        J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
        J.block<3, 3>(0, 0) = Sophus::SO3d::hat(this_point);
        //TODO:并行化
        for (int k = 0; k < voxelmap_->delta_near_.size(); ++k) {
            V3d correspondence_voxel = p_a + voxelmap_->delta_near_[k];
            size_t cur_index = voxelmap_->caculate_hash_index(correspondence_voxel);
            if (voxelmap_->voxel_map_.find(cur_index) != voxelmap_->voxel_map_.end()) {
                V3d p_b = voxelmap_->voxel_map_[cur_index]->centroid_;
                Matrix33 p_b_cov = voxelmap_->voxel_map_[cur_index]->conv_;
                V3d error = p_b - p_a;
                double nn = std::sqrt(voxelmap_->voxel_map_[cur_index]->point_count_);
                Matrix33 mahalanobis_dis = (p_b_cov + cur_pose_.rotationMatrix() * pa_cov *
                                                      cur_pose_.rotationMatrix().transpose()).inverse();
                double chi2_error = error.transpose() * mahalanobis_dis * error;
                Eigen::Matrix<double, 6, 1> new_g = J.transpose() * mahalanobis_dis * error * nn;
                g.block<3, 1>(0, 0) = new_g.tail<3>();
                g.block<3, 1>(3, 0) = new_g.head<3>();
                Eigen::Matrix<double, 6, 6> new_H = J.transpose() * mahalanobis_dis * J * nn;
                H.block<3, 3>(0, 0) += new_H.block<3, 3>(3, 3);
                H.block<3, 3>(6, 6) += new_H.block<3, 3>(0, 0);
                H.block<3, 3>(0, 6) += new_H.block<3, 3>(3, 0);
                H.block<3, 3>(6, 0) += new_H.block<3, 3>(0, 3);
                feat_num_++;
            }
        }
    }
}

void lio::imu_linearize(Eigen::Matrix<double, 15, 15> &H, Eigen::Matrix<double, 15, 1> &g) {//p,v,r

    Sophus::SO3d ori_diff = (last_pose_.inverse() * cur_pose_).so3();
    V3d ori_eror = ori_diff.log();
    Matrix33 jrinv = Sophus::SO3d::leftJacobianInverse(-ori_eror);
    Eigen::Matrix<double, 15, 15> jacobian =
            Eigen::Matrix<double, 15, 15>::Identity();
    jacobian.block<3, 3>(6, 6) = jrinv;

    Eigen::Matrix<double, 15, 1> residual = Eigen::Matrix<double, 15, 1>::Zero();
    residual.block<3, 1>(6, 0) = ori_eror;//,r

    residual.block<3, 1>(3, 0) = cur_vel - last_vel;//v
    //pos;
    residual.block<3, 1>(0, 0) = cur_pose_.translation() - last_pose_.translation();
    residual.block<3, 1>(9, 0) = ba;
    residual.block<3, 1>(12, 0) = bg;

    Eigen::Matrix<double, 15, 15> inv_P = CONV_.block<15, 15>(0, 0).inverse();

    // LOG(INFO) << "inv_P: " << std::endl << inv_P;

    H += jacobian.transpose() * inv_P * jacobian;
    g += jacobian.transpose() * inv_P * residual;

    //(ori_eror);
}

void lio::imu_AHRS(std::shared_ptr<imu> &imu) {
    V3d total_w, total_a;
    total_a.setZero();
    total_w.setZero();
    int n = imu->imu_buff_.size();
    while (!imu->imu_buff_.empty()) {
        sensor_msgs::Imu cur_data = imu->imu_buff_.front();
        imu->imu_buff_.pop_front();
        total_w += V3d(cur_data.angular_velocity.x, cur_data.angular_velocity.y, cur_data.angular_velocity.z);
        total_a += V3d(cur_data.linear_acceleration.x, cur_data.linear_acceleration.y, cur_data.linear_acceleration.z);
    }
    ba = total_a / n;
    bg = total_w / n;
}
