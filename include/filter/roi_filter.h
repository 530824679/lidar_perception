/******************************************************************************/
/*!
File name: roi_filter.h

Description:
This file define class of ROIFilter extract the region to be processed.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_ROI_FILTER_H_
#define _LIDAR_PERCEPTION_ROS_ROI_FILTER_H_

// system include
#include <cfloat>
#include <boost/shared_ptr.hpp>

// local include
#include "common/utils/point3d.h"
#include "common/utils/types.h"

namespace lidar_perception_ros{

    class ROIFilter {
    protected:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<int>> IndicesPtr;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        ROIFilter();
        ROIFilter(ROIParam roi_param);
        virtual ~ROIFilter();

        void PassThough(const PointCloudPtr& input_cloud_ptr, PointCloudPtr& output_cloud_ptr);

        bool InitCompute();
        bool DeinitCompute();

        void SetInputCloud(const PointCloudPtr &cloud);
        void SetFilterLimits(const float &limit_min_x, const float &limit_max_x,
                             const float &limit_min_y, const float &limit_max_y,
                             const float &limit_min_z, const float &limit_max_z);
        void Filter(PointCloud &output_cloud);

        void CopyPointCloud(const PointCloud &input_cloud, const std::vector<int> &indices, PointCloud &output_cloud);

    private:
        std::string filter_field_name_;
        double filter_limit_min_x_;
        double filter_limit_max_x_;
        double filter_limit_min_y_;
        double filter_limit_max_y_;
        double filter_limit_min_z_;
        double filter_limit_max_z_;

        PointCloudPtr point_cloud_ptr_;
        IndicesPtr indices_ptr_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_ROI_FILTER_H_
