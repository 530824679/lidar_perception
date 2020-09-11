/******************************************************************************/
/*!
File name: config_manager.h

Description:
This file define class of config manager to process the parameter of perception module.

Version: 0.1
Create date: 2020.8.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_CONFIG_MANAGER_H_
#define _LIDAR_PERCEPTION_ROS_CONFIG_MANAGER_H_

// system include
#include <fstream>
#include <iostream>
#include <string.h>
#include <chrono>

// local include
#include "common/json/json.h"
#include "common/utils/types.h"

namespace lidar_perception_ros{

    class ConfigManager{
    public:
        ConfigManager();

        ~ConfigManager();

        void SetConfig(const std::string &config_path);

        void SetCalibrateParam(Json::Value param);
        void SetROIParam(Json::Value param);
        void SetVoxelParam(Json::Value param);
        void SetSegmentParam(Json::Value param);
        void SetCurbParam(Json::Value param);
        void SetClusterParam(Json::Value param);
        void SetBBoxParam(Json::Value param);
        void SetTrackerParam(Json::Value param);

        inline CalibrateParam GetCalibrateParam(){ return calibrate_param_;}
        inline ROIParam GetROIParam(){ return roi_param_;};
        inline VoxelParam GetVoxelParam(){ return voxel_param_;};
        inline SegmentParam GetSegmentParam(){ return segment_param_;};
        inline CurbParam GetCurbParam(){ return curb_param_;};
        inline ClusterParam GetClusterParam(){ return cluster_param_;};
        inline BBoxParam GetBBoxParam(){ return bbox_param_;};
        inline TrackerParam GetTrackerParam(){ return tracker_param_;};

    private:
        CalibrateParam calibrate_param_;
        ROIParam roi_param_;
        VoxelParam voxel_param_;
        SegmentParam segment_param_;
        CurbParam curb_param_;
        ClusterParam cluster_param_;
        BBoxParam bbox_param_;
        TrackerParam tracker_param_;

        Json::Reader reader_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_CONFIG_MANAGER_H_
