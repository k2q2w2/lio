//
// Created by nepgear on 2024/2/29.
//
//TODO:icp方法：GICP IMU信息使用

#include "odom_v2.h"
#include <omp.h>
#include "execution"
#include "thread"

bool LO_gicp::LO(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg) {
    std::lock_guard<std::mutex> guard(mtx);
    frame_count_++;

    deskewed_scan->clear();
    timer_.timer("inradius", [&, this]() { Inradius(msg, deskewed_scan); });
    if (frame_count_ < 2) {
        timer_.timer("scanIn2cov", [&, this]() { scan2scanCov(deskewed_scan); });
        timer_.timer("init map", [&, this]() { voxel_map_.add_pointcloud(deskewed_scan); });
        return true;
    }
    //*deskewed_scan = *msg;

    timer_.timer("scanIn2cov", [&, this]() { scan2scanCov(deskewed_scan, 0.5); });
    if (deskewed_scan->size() == 0) return false;
    timer_.timer("GN_step", [&, this]() { GN_optimization(); });
    timer_.timer("addpointcloud2map", [&, this]() {
        double trans = (lidar_pose_.translation() - keyframe_pose_.translation()).norm();
        double rad_s = (lidar_pose_.inverse() * keyframe_pose_).log().norm();
        if (trans < 0.5f || (trans < 0.25f || rad_s < 0.2))
            return;
//#pragma omp parallel for num_threads(num_threads_)//20ms,单线程更快
        keyframe_pose_ = lidar_pose_;
        for (int j = 0; j < deskewed_scan->size(); j++) {
            Eigen::Vector3d p2(deskewed_scan->at(j).x, deskewed_scan->at(j).y, deskewed_scan->at(j).z);
            Eigen::Vector3d pa = lidar_pose_ * p2;
            pcl::PointXYZ new_point = pcl::PointXYZ(pa(0), pa(1), pa(2));
            deskewed_scan->at(j) = new_point;
        }
        voxel_map_.add_pointcloud(deskewed_scan);
    });
    {
        std::vector<double> x;
        x.push_back(frame_count_ * 10);
        x.push_back(lidar_pose_.translation()(0));
        x.push_back(lidar_pose_.translation()(1));
        x.push_back(lidar_pose_.translation()(2));
        x.push_back(lidar_pose_.so3().unit_quaternion().x());
        x.push_back(lidar_pose_.so3().unit_quaternion().y());
        x.push_back(lidar_pose_.so3().unit_quaternion().z());
        x.push_back(lidar_pose_.so3().unit_quaternion().w());
        pos_res_.push_back(x);
    }//记录数据
    std::cout << lidar_pose_.translation().x() << " " << lidar_pose_.translation().y() << " "
              << lidar_pose_.translation().z() << " ";
    return true;
}

void LO_gicp::GN_optimization() {

    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> g;
    int i{};
    int chi2_nums = 0;
    H.setZero();
    g.setZero();
    for (i = 0; i < max_iterations_; i++) {
        icp_point_count = 0;
        int effect_points = 0;
        std::vector<Eigen::Matrix<double, 6, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 6>>> Hs(
                num_threads_ + 1);
        std::vector<Eigen::Matrix<double, 6, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>> gs(
                num_threads_ + 1);
        double cvr = 0;
        int ouliner_points = 0;
        for (int l = 0; l < gs.size(); l++) {
            gs[l].setZero();
            Hs[l].setZero();
        }
        effect_feat_num = 0;

#pragma omp  parallel for num_threads(num_threads_)  //主要耗时部分
        for (int j = 0; j < deskewed_scan->size(); j++) {
            Eigen::Vector3d p_a;
            Eigen::Matrix<double, 3, 6> J;
            Eigen::Vector3d p2(deskewed_scan->at(j).x, deskewed_scan->at(j).y, deskewed_scan->at(j).z);
            p_a = lidar_pose_ * p2;
            //7近邻
            J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
            J.block<3, 3>(0, 0) = Sophus::SO3d::hat(p2);

            for (int k = 0; k < voxel_map_.delta_near_.size(); k++) {
                Eigen::Vector3d near_point = p_a + voxel_map_.delta_near_[k];
                size_t cur_index = caculate_hash_index(near_point);
                if (voxel_map_.voxel_map_.find(cur_index) != voxel_map_.voxel_map_.end()
                    //&& voxel_map_.isSamegrid(near_point, p_b)
                        ) {
                    Eigen::Vector3d p_b = voxel_map_.voxel_map_.find(cur_index)->second->centroid_;
                    Eigen::Matrix3d p_b_cov = voxel_map_.voxel_map_.find(cur_index)->second->conv_;
                    Eigen::Vector3d error = p_b - p_a;
                    double nn = std::sqrt(voxel_map_.voxel_map_[cur_index]->point_count_);
                    Eigen::Matrix3d mahalanobis_dis_ = (p_b_cov +
                                                        lidar_pose_.rotationMatrix() * point_cov_[j] *
                                                        lidar_pose_.rotationMatrix().transpose()
                                                        + Eigen::Matrix3d::Identity() * 1e-3
                    );
                    Eigen::Matrix3d mahalanobis_dis = mahalanobis_dis_.inverse();
                    double chi2_error = error.transpose() * mahalanobis_dis * error;
//                        if ( chi2_error >17.8&&i>5) {
//                            continue;
//                    }
                    effect_points++;
                    gs[omp_get_thread_num()] = gs[omp_get_thread_num()].eval() +
                                               (J.transpose() * mahalanobis_dis * error) * nn;
                    Hs[omp_get_thread_num()] = Hs[omp_get_thread_num()].eval() +
                                               (J.transpose() * mahalanobis_dis *
                                                J) * nn;
                    effect_feat_num++;
                }
            }

        }

        for (int k = 0; k <= num_threads_; k++) {
            H = (H.eval() + Hs[k]);
            g = (g.eval() + gs[k]);
        }
        if (H.determinant() == 0 || H.hasNaN()) {
            LOG(INFO) << "failed to construct H" << frame_count_;
            break;
        }

        if (1) {
            Eigen::Matrix<double, 6, 1> delta_x = -H.inverse() * g;
            if (delta_x.lpNorm<Eigen::Infinity>() < 1e-3 || delta_x.hasNaN()) { break; }
            lidar_pose_ *= Sophus::SE3d(Sophus::SO3d::exp(delta_x.head<3>()),
                                        delta_x.tail<3>());

        }
        //std::cout << lidar_pose_.translation() << std::endl;
    }
    std::cout << "effect_feat_num" << effect_feat_num << " ";
    std::cout << "iteration time:" << i << " ";

    //std::cout << "lidar——pose:" << lidar_pose_.translation() << " ";
}


void LO_gicp::scan2scanCov(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg) {
    point_cov_.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr newmsg = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
    voxelmap temp_map(false, 0.5f);
    temp_map.add_pointcloud(msg);//TODO:代码不能复用，需要分解出协方差和点云操作
    point_cov_.reserve(temp_map.voxel_map_.size());
    newmsg->reserve(temp_map.voxel_map_.size());
    int poinst_with_cov = 0;
    std::for_each(std::execution::seq, temp_map.voxel_map_.begin(), temp_map.voxel_map_.end(),
                  [&, this](const auto &v_begin) {

                      pcl::PointXYZ n_p = pcl::PointXYZ(v_begin.second->centroid_(0), v_begin.second->centroid_(1),
                                                        v_begin.second->centroid_(2));
                      newmsg->push_back(n_p);
                      point_cov_.push_back(v_begin.second->conv_);
                      poinst_with_cov++;

                  });//TODO：处理冲突
//    std::cout << poinst_with_cov << std::endl;
//    for (auto &i: temp_map.voxel_map_) {
//        std::cout << i.second->f_ver << std::endl;
//    }
    pcl::copyPointCloud(*newmsg, *msg);//点云复制请使用该函数，为深拷贝
}

void LO_gicp::scan2scanCov(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, double res) {
    point_cov_.clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr newmsg = pcl::PointCloud<pcl::PointXYZ>::Ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
    voxelmap temp_map(false, res);
    temp_map.add_pointcloud(msg);//TODO:代码不能复用，需要分解出协方差和点云操作
    point_cov_.reserve(temp_map.voxel_map_.size());
    newmsg->reserve(temp_map.voxel_map_.size());
    int poinst_with_cov = 0, effect_cov_num = 0;
    std::for_each(std::execution::seq, temp_map.voxel_map_.begin(), temp_map.voxel_map_.end(),
                  [&, this](const auto &v_begin) {
                      pcl::PointXYZ n_p = pcl::PointXYZ(v_begin.second->centroid_(0), v_begin.second->centroid_(1),
                                                        v_begin.second->centroid_(2));
                      std::vector<Eigen::Vector3d> result;
                      temp_map.KNN_search(v_begin.second->centroid_, 20, 2.0f, result);//KNN计算
                      //TODO：pca计算平面方差

                      if (result.size() >= 6) {
                          //enought points for pca;
                          Eigen::MatrixXd neighbors(4, result.size());
                          for (int j = 0; j < result.size(); j++) {
                              neighbors.col(j) = Eigen::Vector4d(result[j](0), result[j](1), result[j](2), 1);
                          }
                          neighbors.colwise() -= neighbors.rowwise().mean().eval();
                          Eigen::Matrix4d cov = neighbors * neighbors.transpose() / result.size();
                          Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0),
                                                                Eigen::ComputeFullU | Eigen::ComputeFullV);
                          Eigen::Vector3d values;
                          values = Eigen::Vector3d(1, 1, 1e-3);
                          Eigen::Matrix3d conv = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
                          point_cov_.push_back(conv);
                          newmsg->push_back(n_p);
                          poinst_with_cov++;
                          if (conv == Eigen::Matrix3d::Zero()) {
                              effect_cov_num++;
                          }
                      }
                  });
    std::cout << effect_cov_num << " " << poinst_with_cov << std::endl;
    pcl::copyPointCloud(*newmsg, *msg);
}

size_t LO_gicp::caculate_hash_index(Eigen::Vector3d &p) {
    double xyz[3];
    for (int i = 0; i < 3; i++) {
        xyz[i] = p(i) / resolution;
        if (xyz[i] < 0) xyz[i] -= 1.0;
    }
    size_t x = static_cast<size_t>(xyz[0]);
    size_t y = static_cast<size_t>(xyz[1]);
    size_t z = static_cast<size_t>(xyz[2]);
    return (((z * HASH_P) % MAX_N + y) * HASH_P) % MAX_N + x;
}

LO_gicp::LO_gicp() {
    deskewed_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

void LO_gicp::Inradius(pcl::PointCloud<pcl::PointXYZ>::Ptr &Input, pcl::PointCloud<pcl::PointXYZ>::Ptr &Ouput) {
//#pragma omp parallel for
    for (int i = 0; i < Input->size(); i++) {
        Eigen::Vector3f cur_p = Eigen::Vector3f(Input->at(i).x,
                                                Input->at(i).y, Input->at(i).z);
        if (cur_p.norm() <= 150.0f && cur_p.norm() >= 0.5f) {
            Ouput->push_back(pcl::PointXYZ(cur_p(0), cur_p(1), cur_p(2)));
        }
    }
}

void LO_gicp::CauchyLossFunction(const double e,
                                 const double delta,
                                 Eigen::Vector3d &rho) {
    double dsqr = delta * delta;
    if (e <= dsqr) {  // inlier
        rho[0] = e;
        rho[1] = 1.;
        rho[2] = 0.;
    } else {                   // outlier
        double sqrte = sqrt(e);  // absolut value of the error
        // rho(e)   = 2 * delta * e^(1/2) - delta^2
        rho[0] = 2 * sqrte * delta - dsqr;
        // rho'(e)  = delta / sqrt(e)
        rho[1] = delta / sqrte;
        // rho''(e) = -1 / (2*e^(3/2)) = -1/2 * (delta/e) / e
        rho[2] = -0.5 * rho[1] / e;
    }
}