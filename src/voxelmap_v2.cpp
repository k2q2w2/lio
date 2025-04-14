//
// Created by nepgear on 2024/4/16.
//
#include "voxelmap_v2.h"

namespace npg_v2 {
    void Voxelmap::AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &msg, bool cov_flag) {
        std::vector<size_t> key_array(msg->size());
        std::unordered_map<size_t, std::vector<Eigen::Vector3d>> des;
        key_array.resize(msg->size());
        for (int i = 0; i < msg->size(); i++) {
            Eigen::Vector3d this_point = (msg->at(i).getVector3fMap().template cast<double>());
            size_t index = caculate_hash_index(this_point);
            key_array[i] = index;
        }
        std::set<size_t> key_map(key_array.begin(), key_array.end());
        des.reserve(key_map.size());
        std::for_each(std::execution::seq, key_map.begin(), key_map.end(), [&, this](auto &i) {
            if (voxel_map_.find(i) == voxel_map_.end()) {
                auto new_grid = std::make_shared<grid>();
                new_grid->hash_idx_ = i;
                new_grid->point_count_ = 0;
                new_grid->points_sum_ = Eigen::Vector3d::Zero();
                new_grid->cov_sum_ = Eigen::Matrix3d::Zero();;
                new_grid->conv_ = Eigen::Matrix3d::Zero();//默认是0
                new_grid->centroid_ = Eigen::Vector3d::Zero();
                voxel_map_.insert(std::make_pair(i, new_grid));
            }
            std::vector<Eigen::Vector3d> tr_empty_cache;
            des.insert(std::make_pair(i, tr_empty_cache));
        });//插入含有全量key的空grid
        for (int i = 0; i < key_array.size(); i++) {
            Eigen::Vector3d res = (msg->at(i).getVector3fMap().template cast<double>());
            des[key_array[i]].emplace_back(res);
        }
        std::for_each(std::execution::seq, des.begin(), des.end(), [&, this](auto &i) {
                          for (int j = 0; j < i.second.size(); j++) {
                              if (voxel_map_[i.first]->point_count_ >= 50) {
                                  if (voxel_map_[i.first]->first_index >= 50) voxel_map_[i.first]->first_index -= 50;
                                  Eigen::Vector3d last_pt = voxel_map_[i.first]->points_array_[voxel_map_[i.first]->first_index];
                                  voxel_map_[i.first]->points_array_[voxel_map_[i.first]->first_index] = i.second[j];
                                  voxel_map_[i.first]->points_sum_ -= (last_pt - i.second[j]);
                                  voxel_map_[i.first]->first_index++;
                                  continue;
                              };
                              voxel_map_[i.first]->points_array_.push_back(i.second[j]);
                              voxel_map_[i.first]->point_count_ += 1;
                              voxel_map_[i.first]->points_sum_ = voxel_map_[i.first]->points_sum_.eval() + i.second[j];
                          }
                      }
        );//性能主要占用
        std::for_each(std::execution::seq, key_map.begin(), key_map.end(),
                      [&, this](auto &i) {
                          voxel_map_[i]->centroid_ =
                                  voxel_map_[i]->points_sum_ / voxel_map_[i]->point_count_;
                          if (cov_flag)
                              caculate_conc(voxel_map_[i]);
                      }
        );
    }

    Voxelmap::Voxelmap(bool near7_flag, double res) : resolution(res) {
        if (near7_flag) {
            delta_near_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            delta_near_.push_back(Eigen::Vector3d(resolution, 0.0, 0.0));
            delta_near_.push_back(Eigen::Vector3d(0.0, resolution, 0.0));
            delta_near_.push_back(Eigen::Vector3d(0.0, 0.0, resolution));
            delta_near_.push_back(Eigen::Vector3d(-resolution, 0.0, 0.0));
            delta_near_.push_back(Eigen::Vector3d(0.0, -resolution, 0.0));
            delta_near_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution));
        } else {
            for (int x = -1; x <= 1; x++) {
                for (int y = -1; y <= 1; y++) {
                    for (int z = -1; z <= 1; z++) {
                        delta_near_.push_back(Eigen::Vector3d(x * resolution, y * resolution, z * resolution));
                    }
                }
            }
        }
    }

    size_t Voxelmap::caculate_hash_index(Eigen::Vector3d &p) {
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

    size_t Voxelmap::caculate_hash_index(Eigen::Vector3d &&p) {
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

    void Voxelmap::caculate_conc(std::shared_ptr<grid> &this_grid) {
        if (this_grid->point_count_ >= 6) {
            Eigen::MatrixXd neighbors(4, this_grid->point_count_), nn(4, this_grid->point_count_);
            for (int i = 0; i < this_grid->point_count_; i++) {
                neighbors.col(i) = Eigen::Vector4d(this_grid->points_array_[i](0), this_grid->points_array_[i](1),
                                                   this_grid->points_array_[i](2), 1);
            }
            neighbors.colwise() -= neighbors.rowwise().mean().eval();
            Eigen::Matrix<double, 4, 4> cov =
                    neighbors.eval() * neighbors.transpose().eval() / (this_grid->point_count_);
            Eigen::Vector3d sigma(1, 1, 1e-3);
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov.block<3, 3>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d res_conv = svd.matrixU() * sigma.asDiagonal() * svd.matrixV().transpose();
            this_grid->conv_ = res_conv;
            this_grid->is_valid_ = true;
            points_with_covs++;
        } else {
            this_grid->is_valid_ = false;
        }
    }

    bool Voxelmap::KNN_search(Eigen::Vector3d &point, size_t n, double range, std::vector<Eigen::Vector3d> &result) {
        auto range2 = range * range;
        Eigen::Vector3d n_p;
        std::vector<knn_point> point_dist;
        point_dist.reserve(delta_near_.size() * n);
        for (const auto &i: delta_near_) {//27near
            n_p = point + i;
            size_t hash_idx = caculate_hash_index(n_p);
            if (voxel_map_.find(hash_idx) != voxel_map_.end()) {
                Eigen::Vector3d this_point_ = voxel_map_[hash_idx]->centroid_;
                //voxel_map_[hash_idx]->points_array_.pop();
                auto temp_dist = (this_point_ - point);
                if (temp_dist.norm() < range2) {
                    point_dist.push_back(knn_point(this_point_, temp_dist.norm()));
                }
            }
        }
        if (point_dist.size() == 0) {
            return false;
        }
        if (point_dist.size() > n) {
            //裁剪，选择n个元素
            std::nth_element(point_dist.begin(), point_dist.begin() + n - 1, point_dist.end());
            for (int i = 0; i < n; i++) {
                result.emplace_back(point_dist[i].point_);
            }
            return true;
        }
        for (auto &i: point_dist) {
            result.emplace_back(i.point_);
        }
        return true;
    }
}