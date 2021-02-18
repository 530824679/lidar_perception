//
/******************************************************************************/
/*!
File name: object_segment.h

Description:
This file define class of Grid to rasterize point cloud

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_GRID_H_
#define _LIDAR_PERCEPTION_ROS_GRID_H_

// json include
#include "common/json/json.h"

// math include
#include <math.h>

// local include
#include "manager/config_manager.h"
#include "common/utils/point3d.h"
#include "common/utils/types.h"

namespace lidar_perception_ros{

    class Grid{
    protected:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::vector<int> Indices;

    public:
        Grid();
        ~Grid();

        bool init_;
        int type_;
        int points_num_;
        float min_height_;
        float max_height_;
        float height_diff_;
        float intensity_ave_;

        bool is_roadside_;
        PointCloud grid_cloud_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_GRID_H_
