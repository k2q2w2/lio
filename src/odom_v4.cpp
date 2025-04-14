//
// Created by nepgear on 2024/9/29.
//
#include "odom_v4.h"
namespace gear_lio {
    void lio::pipeline(const sensor_msgs::PointCloud2::Ptr &msg) {
        /*工作流程：
 * TODO：
 * 1.初始化订阅话题+外参设置（imu+lidar）-> 2.imu零漂估计
 *                             loop:->1. 点云预处理(voxel_downsample)+radius_filter
 *                                  ->2. 地图初始化
 *                                  ->3. nominate_update
 *                                  ->4. gicp线性化
 *                                  ->5. imu线性化
 *                                  ->6. 优化
 *                                  ->7. 关键帧更新
 * */
        std::lock_guard<std::mutex> lock(lio_guard_);
        if(msg->width*msg->height==0)
        {
            LOG(ERROR)<<"no data"<<"\n";
            return;
        }
        cur_time = msg->header.stamp.toSec();
        if(cur_time<last_time)
        {
            LOG(ERROR)<<"wrong in time"<<"\n";
            return ;
        }
        if(!field_check_flag_)
        {
           field_check_flag_= true;
           field_check(msg);
            LOG(INFO)<<"success field check"<<"\n";
        }
        if(!imu_AHRS_flag_)
        {
            if(imu_->imu_buff_.size()>10) {
                imu_AHRS_flag_ = true;
                imu_AHRS(imu_);
                LOG(INFO) << "success initialize imu" << "\n";
            }else
            {
                return ;
            }
        }
        frame_count++;
        /***downsample***/
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr distorted_pc(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_cov(new pcl::PointCloud<pcl::PointXYZ>),pc(new pcl::PointCloud<pcl::PointXYZ>);
        /***get pointcloud & radius filter***/
        preprocess(msg,point_cloud);
        /***distorted***/
        distort(point_cloud,distorted_pc);
        /***filter***/
        filter(distorted_pc,pc);//去除时间+下采样
        /***map filled***/
        if(frame_count<10)
        {
            voxelmap_->AddPointCloud(pc, false);
        }
        /***covariance***/
        compute_covariance(pc,cloud_ptr_cov,covariance_);//TODO：点云与点云后的合并？
        /***predict***/
        predict();
        /***optimization***/
        linearize();
        optimization();
        /***keyframe update***/
        Keyframe_update();
        /***update time***/
        last_time=cur_time;
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
                lidar_type_=velodyne;
            }
            //TODO:add livox lidar field
        }
    }
    void lio::imu_AHRS(std::shared_ptr<imu> &imu) {
        int data_size = imu->imu_buff_.size();
        V3d angular_sum{},accel_sum{};
        for(const auto &i:imu->imu_buff_)
        {
            angular_sum+=V3d(i.angular_velocity.x,i.angular_velocity.y,i.angular_velocity.z);
            accel_sum+=V3d(i.linear_acceleration.x,i.linear_acceleration.y,i.linear_acceleration.z);
        }
        imu_angualr_bias = angular_sum/data_size;
        imu_accel_bias = accel_sum/data_size;
    }
    void lio::preprocess(const sensor_msgs::PointCloud2::Ptr &msg, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &cloud_out) {
        size_t point_cloud_size = msg->height * msg->width;
        cloud_out->clear();
        cloud_out->reserve(point_cloud_size);
        size_t point_step = msg->point_step;
        pcl::PointXYZINormal new_point;
        for (size_t i = 0; i < point_cloud_size; i++) {
            new_point.x = *(float *) (&msg->data[0] + point_field_offset_.offset_x + point_step * i);
            new_point.y = *(float *) (&msg->data[0] + point_field_offset_.offset_y + point_step * i);
            new_point.z = *(float *) (&msg->data[0] + point_field_offset_.offset_z + point_step * i);
            float radius_2 = compute_radius<pcl::PointXYZINormal>(new_point);
            if(radius_filter_flag_&&maximum_radius_2<radius_2) continue;
            if (lidar_type_ == velodyne) {
                new_point.curvature =
                        *(float *) (&msg->data[0] + point_field_offset_.offset_time + point_step * i);//unit:s
            }
            cloud_out->push_back(new_point);
        }
    }
    void lio::distort(pcl::PointCloud<pcl::PointXYZINormal>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud) {
        int imu_data_size = imu_->imu_buff_.size();
        auto sort_func = [](const sensor_msgs::Imu &imu1,const sensor_msgs::Imu &imu2)->bool{
            return imu1.header.stamp.toSec()>imu2.header.stamp.toSec();
        };
        std::vector<sensor_msgs::Imu> imu_puff(imu_data_size+1);
        std::partial_sort_copy(std::execution::par,imu_->imu_buff_.begin(),imu_->imu_buff_.end(),
                               imu_puff.begin()+1,imu_puff.end(),sort_func);

        std::function<bool(pcl::PointXYZINormal &,pcl::PointXYZINormal &)> time_cmp;
        if(lidar_type_==velodyne)
        {
            time_cmp = [](pcl::PointXYZINormal &p1,pcl::PointXYZINormal &p2)->bool {
                return p1.curvature<p2.curvature;
            };
        }
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr distort_cloud;
        std::partial_sort_copy(in_cloud->begin(),in_cloud->end(),
                               distort_cloud->begin(),distort_cloud->end(),time_cmp);
        //sort in time
        double last_imu_time = imu_->last_imu_data_.header.stamp.toSec(),pointcloud_index=0;
        sensor_msgs::Imu last_imu = imu_->last_imu_data_;
        imu_puff[0] = last_imu;
        std::reverse(imu_puff.begin(),imu_puff.end());
        last_imu = imu_puff[imu_puff.size()-1];
        Mat33 R_imu_acc;
        R_imu_acc.setIdentity();
        V3d T_imu_acc,V_imu_acc;
        T_imu_acc.setZero(),V_imu_acc.setZero();
        pcl::PointXYZI new_p;
        for(int i=imu_puff.size()-2;i>=0;i--) {
        //smooth:
        auto v = imu_puff[i];
        V3d linear_accel = V3d(last_imu.linear_acceleration.x/2,last_imu.linear_acceleration.y/2,last_imu.linear_acceleration.z/2)+
                V3d(v.linear_acceleration.x/2,v.linear_acceleration.y/2,v.linear_acceleration.z/2)-imu_accel_bias;
        V3d angular = V3d(last_imu.angular_velocity.x/2,last_imu.angular_velocity.y/2,last_imu.angular_velocity.z/2)+
                V3d(v.angular_velocity.x/2,v.angular_velocity.y/2,v.angular_velocity.z/2)-imu_angualr_bias;
        double cur_imu_time = v.header.stamp.toSec();
            while(pointcloud_index<distort_cloud->height*distort_cloud->width){
                double dt = -distort_cloud->at(pointcloud_index).curvature+last_imu_time;
                if(dt<0){
                    continue;
                }
                if(distort_cloud->at(pointcloud_index).curvature<cur_imu_time){
                    break;
                }
                Mat33 R_t = R_imu_acc*Sophus::SO3d::exp(dt*angular).matrix();
                V3d T_t = T_imu_acc+0.5*linear_accel*dt*dt+dt*V_imu_acc;
                V3d n_p = R_t*(V3d(distort_cloud->at(pointcloud_index).x,distort_cloud->at(pointcloud_index).y,distort_cloud->at(pointcloud_index).z))
                        +T_t;
                //add distort
                new_p.x=n_p.x(),new_p.y=n_p.y(),new_p.z=n_p.z();
                out_cloud->push_back(new_p);
            pointcloud_index++;
            }
            //update imu R,T
            double imu_dt = last_imu_time-cur_imu_time;
            T_imu_acc = T_imu_acc.eval() + V_imu_acc*imu_dt+0.5*imu_dt*imu_dt*linear_accel;
            R_imu_acc = R_imu_acc.eval()*Sophus::SO3d::exp(imu_dt*angular).matrix();
            V_imu_acc = V_imu_acc.eval() + linear_accel*imu_dt;
            last_imu=v;
            last_imu_time = cur_imu_time;
        }
    }
    void lio::filter(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) {
        /*1. tf转换—>imu坐标系
         *2.
         * */
    }
}