#include "odom.h"

namespace npg {
/*
    void icp::registerlaserdata(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
        std::lock_guard<std::mutex> guard(mtx);
        pcl::PointCloud<livox_ros_driver::CustomPoint>::Ptr cloud(new pcl::PointCloud<livox_ros_driver::CustomPoint>);
        auto cur_time = msg->header.stamp.toSec();
//#pragma omp parallel for num_threads(8)
        for (int i = 0; i < msg->point_num; i++) {
            livox_ros_driver::CustomPoint p;
            p.x = msg->points[i].x;
            p.y = msg->points[i].y;
            p.z = msg->points[i].z;
            p.reflectivity = msg->points[i].reflectivity;
            p.offset_time = msg->points[i].offset_time;
            cloud->push_back(p);
        }
        //deskewedPoints(cloud, cur_time);//点云去畸变
        *deskewed_scan = *cloud;//TODO：debug模式，不测试去畸变功能
        //构造kdtree,判断是否为第一帧雷达数据
        if (!first_scan_valid_) {
            this_scan_in_kdtree.scan_ = deskewed_scan;
            last_scan_ = deskewed_scan;
            index_ = std::make_shared<my_kd_tree>(3, this_scan_in_kdtree);
            index_->buildIndex();
            first_scan_valid_ = true;
            return;
        }
        lidar_pose_ = GaussianNewton();
        *last_scan_ = *last_scan_ + *deskewed_scan;
        *index_->dataset_.scan_ = *last_scan_;//前后帧替换点云
        index_->buildIndex();//重新构建kd_tree
        //构造kdtree
        //搜索最近邻，构造icp error function，使用L—M方法和IMU测量值作为初值求解
    }
*/
    void icp::registerlaserdata(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg) {
        std::lock_guard<std::mutex> guard(mtx);
        frame_count++;
//        if (frame_count >= 30) {
//            using namespace std;
//            //cout << "finish 30 fps" << endl;
//            LOG(INFO) << "kd_tree get size:" << last_scan_->size() << endl;
//            ros::shutdown();
//            return;
//        }
        *deskewed_scan = *msg;//TODO：debug模式，不测试去畸变功能
        //构造kdtree,判断是否为第一帧雷达数据
        if (!first_scan_valid_) {
            *last_scan_ = *deskewed_scan;
            this_scan_in_kdtree.scan_ = last_scan_;
            index_ = std::make_shared<my_kd_tree>(3, this_scan_in_kdtree, 50);
            index_->buildIndex();
            first_scan_valid_ = true;
            last_poses_ = lidar_pose_;
            return;
        }
        auto tic = std::chrono::system_clock::now();

        lidar_pose_ *= GaussianNewton();
        delta_x_ = lidar_pose_.translation()-last_poses_.translation();
        auto delta_r = lidar_pose_.so3().log()-last_poses_.so3().log();
        {
            std::vector<double> x;
            x.push_back(frame_count*10);
            x.push_back(lidar_pose_.translation()(0));
            x.push_back(lidar_pose_.translation()(1));
            x.push_back(lidar_pose_.translation()(2));
            x.push_back(lidar_pose_.so3().unit_quaternion().x());
            x.push_back(lidar_pose_.so3().unit_quaternion().y());
            x.push_back(lidar_pose_.so3().unit_quaternion().z());
            x.push_back(lidar_pose_.so3().unit_quaternion().w());
            pos_res_.push_back(x);
        }//记录数据
        std::cout << "icp count is :" << icp_point_count << "\t";
        auto tic_end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds2 = tic_end - tic;
        //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        std::cout << "time in gn step is:" << elapsed_seconds2.count() << "\t";
        auto start = std::chrono::system_clock::now();
        //TODO：添加关键帧判断
        if (frame_count >= 10) {
            if (delta_x_.norm() >= 0.5 || delta_r.norm() >= 0.18
             ) {
                keyframe_count++;
                *last_scan_ = *deskewed_scan;//TODO:shit hill,你新的点云不变换的？
                this_scan_in_kdtree.scan_ = last_scan_;
                index_ = std::make_shared<my_kd_tree>(3, this_scan_in_kdtree, 50);
                index_->buildIndex();
                last_poses_ = lidar_pose_;
            }
        } else {
            *last_scan_ = *deskewed_scan+*last_scan_;//
            this_scan_in_kdtree.scan_ = last_scan_;
            index_ = std::make_shared<my_kd_tree>(3, this_scan_in_kdtree, 50);
            index_->buildIndex();
        }
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        std::cout << "key_frame_count is " << keyframe_count << "\t";
    }

    Sophus::SE3d icp::GaussianNewton() {//GN方法准确，1.15已经验证
        // TODO：速度最慢的地方，大概率是k近邻查找慢,solve：矩阵相加乘法较慢
        //TODO：优化代码，与第几个近邻没有关系，需要控制迭代次数，改变voxel_grid size;
        //todo: 优化代码，删除不必要的部分
        Eigen::Matrix<double, 3, 6> J;
        Eigen::Matrix<double, 6, 6> H;
        Eigen::Matrix<double, 6, 1> g;
        Sophus::SE3d pose_gn;//initial value set here
        pose_gn = pose_gn.inverse();
        auto start = std::chrono::high_resolution_clock::now();
        double total_count_point = 0;
        double total_time_in_HGCOST = 0;
        double current_cost = 0, last_cost = MAXFLOAT, iteration_times = 0, temp_max_min_icp_distance_ = max_min_icp_distance_;
        for (int i = 0; i < max_iterations_; i++) {
            current_cost = 0;
            J.setZero();
            H.setZero();
            g.setZero();//tbb 优化 构造函数过程
            icp_point_count = 0;
#pragma omp declare reduction (merge_H : Eigen::Matrix<double, 6, 6> : omp_out += omp_in)
#pragma omp declare reduction (merge_g : Eigen::Matrix<double, 6, 1> : omp_out += omp_in)
#pragma omp parallel  for num_threads(8) reduction(merge_H:H) reduction(merge_g:g) reduction(+:current_cost)
            for (int j = 0; j < deskewed_scan->size(); j++) {
                //TODO:construct icp constraint
                //nearest neighbor find
                Eigen::Vector3d p2(deskewed_scan->at(j).x, deskewed_scan->at(j).y, deskewed_scan->at(j).z);
                auto p3 = pose_gn * p2.eval();
                //construct transform;
                const size_t num_result = 1;
                size_t ret = 0;
                double out_distance = 0;
                nanoflann::KNNResultSet<double> resultSet(num_result);
                double query_point[3] = {p3(0), p3(1), p3(2)};
                resultSet.init(&ret, &out_distance);
                index_->findNeighbors(resultSet, query_point);
                if (out_distance > temp_max_min_icp_distance_) {
                    continue;
                }
                icp_point_count++;
                J.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
                J.block<3, 3>(0, 0) = -pose_gn.so3().matrix() * Sophus::SO3d::hat(p3);
                Eigen::Vector3d error = {p3(0) - last_scan_->at(ret).x,
                                         p3(1) - last_scan_->at(ret).y,
                                         p3(2) - last_scan_->at(ret).z};
                auto error_norm = error.norm();

                H += J.transpose() * J;
                current_cost = current_cost + error_norm;
                g += -J.transpose() * error;

            }

            //LOG(INFO) << "get " << icp_point_count;
            current_cost = current_cost / icp_point_count;
            double cost_change = last_cost - current_cost;
            if (H.determinant() == 0) {
                //LOG(INFO) << "failed to construct H" << current_cost;
                break;
            }
            if (/*cost_change > 1e-4*/1) {
                last_cost = current_cost;
                ll_cost = last_cost;

                //LOG(INFO) << "successfully gradient descent! current cost is:" << current_cost;
                Eigen::Matrix<double, 6, 1> delta_x = H.inverse() * g;
                if(delta_x.lpNorm<Eigen::Infinity>()<1e-3) break;
                pose_gn = Sophus::SE3d(pose_gn.so3() * Sophus::SO3d::exp(delta_x.head<3>()),
                                       pose_gn.translation() + delta_x.tail<3>());
                temp_max_min_icp_distance_ = temp_max_min_icp_distance_ * factor_;
                //LOG(INFO) << "get new pose:" << "\n" << pose_gn.inverse().translation();
            } else {
                LOG(INFO) << "finish descent gradient" << current_cost;
                LOG(INFO) << "iteration time: " << i;
                break;
            }
        }
        //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
//        LOG(INFO) << elapsed_seconds.count();
        return
                pose_gn;
    }

/*
    void icp::deskewedPoints(pcl::PointCloud<livox_ros_driver::CustomPoint>::Ptr &pc, double cur_time) {
        auto point_time_cmp = [](livox_ros_driver::CustomPoint &a, livox_ros_driver::CustomPoint &b) {
            return a.offset_time < b.offset_time;
        };
        std::partial_sort_copy(pc->begin(), pc->end(), deskewed_scan->begin(),
                               deskewed_scan->end(), point_time_cmp);
        imu_index = 0;
        using namespace Eigen;
        double timebase = cur_time;
        R_scans_ = Matrix<double, 3, 3>::Identity();//构造SO3
        P_scans_.setZero();
        V_scans_.setZero();
        for (int i = 0; i < deskewed_scan->size(); ++i) {
            double t = cur_time + deskewed_scan->at(i).offset_time * 1e-9f;
            while (imu_buffer.size() > i && imu_buffer[imu_index].time < t) {
                double delta_t = imu_buffer[imu_index].time - cur_time;
                if (delta_t <= 0) continue;
                P_scans_ = P_scans_ + V_scans_ * delta_t + 0.5 * imu_buffer[imu_index].accel * delta_t * delta_t;
                V_scans_ = V_scans_.eval() + R_scans_ * imu_buffer[imu_index].accel * delta_t;
                R_scans_ = R_scans_.eval() * Sophus::SO3d::exp(imu_buffer[imu_index].gyro * delta_t).matrix();
                cur_time += delta_t;
                imu_index++;//update p,v,R;插值处理p
            }
            double compensate_time = 0;
            if (i == 0) compensate_time = t - timebase;
            else {
                compensate_time = t - imu_buffer[imu_index - 1].time;
            }
            P_scans_ = P_scans_.eval() + (imu_buffer[imu_index].accel - imu_buffer[imu_index - 1].accel) /
                                         (6 * (imu_buffer[imu_index].time - imu_buffer[imu_index - 1].time)) *
                                         compensate_time * compensate_time * compensate_time;//三阶前向传播
            Vector3d p_p = Vector3d(deskewed_scan->at(i).x, deskewed_scan->at(i).y, deskewed_scan->at(i).z);
            p_p = R_scans_.transpose() * (p_p.eval() - P_scans_);
            deskewed_scan->at(i).x = p_p(0);
            deskewed_scan->at(i).y = p_p(1);
            deskewed_scan->at(i).z = p_p(2);
        }
    }
*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr icp::icp_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg) {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(msg);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outputPointcloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        sor.filter(*outputPointcloud);
        return outputPointcloud;
    }

}
