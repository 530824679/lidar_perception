/******************************************************************************/
/*!
File name: lidar_process.h

Description:
This file define class of lidar process to realize lidar perception module.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _PERCEPTION_ROS_LIDAR_PROCESS_H_
#define _PERCEPTION_ROS_LIDAR_PROCESS_H_

// pcl include
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

// ros include
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// system include
#include <fstream>
#include <iostream>
#include <string.h>
#include <chrono>

// local include
#include "common/json/json.h"
#include "common/utils/types.h"
#include "common/utils/point3d.h"

#include "filter/roi_filter.h"
#include "filter/voxel_filter.h"
#include "filter/outlier_filter.h"
#include "segment/object_segment.h"
#include "segment/object_cluster.h"
#include "detect/curb_detect.h"
#include "builder/bbox_fitting.h"
#include "builder/bbox_fitting_lshape.h"
#include "tracker/tracking.h"
#include "manager/config_manager.h"

namespace lidar_perception_ros {

    class LidarProcess {
    public:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        LidarProcess();

        ~LidarProcess();

        bool Init(const std::string &config_path);

        void ProcessLidarData(const sensor_msgs::PointCloud2ConstPtr& p_Horizon_ptr);

        ros::NodeHandle node_;
        ros::Subscriber lidar_subscriber_;

    private:
        void ConversionData(const sensor_msgs::PointCloud2::ConstPtr& input, PointCloudPtr& input_cloud_ptr);

        void ProcessPointCloud(const PointCloudPtr& input_cloud_ptr);

        void CalibratePointCloud(PointCloudPtr& input_cloud_ptr);

    private:
        // Sub module ptr
        std::shared_ptr<ROIFilter> roi_filter_;
        std::shared_ptr<OutlierFilter> outlier_filter_;
        std::shared_ptr<VoxelFilter> voxel_filter_;
        std::shared_ptr<CurbDetect> curb_detect_;
        std::shared_ptr<Segment> segment_;
        std::shared_ptr<Cluster> object_cluster_;
        std::shared_ptr<LShapeBBoxEstimator> bbox_fitting_;
        std::shared_ptr<Tracking> tracking_;

        // Config
        ConfigManager config_manager_;
        Eigen::Matrix4d extrinsics_;

        pcl::visualization::PCLVisualizer::Ptr viewer_;
    };
}
#endif //_PERCEPTION_ROS_LIDAR_PROCESS_H_
