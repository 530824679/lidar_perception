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

#ifndef _LIDAR_PERCEPTION_ROS_OUTLIER_FILTER_H_
#define _LIDAR_PERCEPTION_ROS_OUTLIER_FILTER_H_

// system include
#include <cmath>
#include <vector>
#include <memory>

// local include
#include "common/utils/point3d.h"

namespace lidar_perception_ros{

    class OutlierFilter
    {
    public:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        OutlierFilter() = default;
        virtual ~OutlierFilter(){};

        void OutlierRemove(const PointCloudPtr& input_cloud_ptr, PointCloudPtr &out_cloud_ptr);
        void InvalidRemove(PointCloudPtr& input_cloud_ptr);
    };

}
#endif //_LIDAR_PERCEPTION_ROS_OUTLIER_FILTER_H_
