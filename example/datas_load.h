/******************************************************************************/
/*!
File name: outlier_filter.h

Description:
This file define class of OutlierFilter to remove the discrete group of points.

Version: 0.1
Create date: 2020.7.20
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/


#ifndef _LIDAR_PERCEPTION_ROS_DATAS_LOAD_H_
#define _LIDAR_PERCEPTION_ROS_DATAS_LOAD_H_

#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>

#include "common/utils/point3d.h"
#include "common/utils/types.h"
#include "manager/config_manager.h"

namespace lidar_perception_ros {

    typedef typename std::vector<PointXYZI<float>> PointCloud;
    typedef typename std::shared_ptr<std::vector<int>> IndicesPtr;
    typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    class DATASLOAD {
    public:
        DATASLOAD()= default;
        DATASLOAD(string pcd_path="");
        ~DATASLOAD();

        void LoadRosbag(ros::NodeHandle& node, ros::Subscriber& subscriber, void(*CallBackCloud)(const sensor_msgs::PointCloud2ConstPtr& p_Horizon_ptr));
        void LoadPcd(void(*PcdData)(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr));

    private:
        string pcd_path_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_DATAS_LOAD_H_
