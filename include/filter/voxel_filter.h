/******************************************************************************/
/*!
File name: down_sampling.h

Description:
This file define class of VoxelFilter to down sampling the point cloud.

Version: 0.1
Create date: 2020.7.20
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_VOXEL_FILTER_H
#define _LIDAR_PERCEPTION_ROS_VOXEL_FILTER_H

// pcl include
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

// system include
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <vector>

// local include
#include "common/utils/types.h"
#include "common/utils/point3d.h"

namespace lidar_perception_ros{

    class VoxelFilter{
    protected:
        typedef typename std::vector<PointXYZI<float>> PointCloud;

        struct PointKDistance{
            PointXYZI<float> point_;
            float distance_;
        };

        struct PointCloudIndexIdx{
            unsigned int idx_;
            unsigned int point_cloud_index_;

            PointCloudIndexIdx(unsigned int idx_, unsigned int point_cloud_index_) : idx_(idx_), point_cloud_index_(point_cloud_index_) {}
            bool operator < (const PointCloudIndexIdx &p) const { return (idx_ < p.idx_); }
        };

    public:
        VoxelFilter() = default;
        VoxelFilter(VoxelParam roi_param);
        ~VoxelFilter(){};

        void VoxelProcess(PointCloud& input_point_cloud, PointCloud& out_point_cloud);

    private:
        float EucliDistance(PointXYZI<float> &point1, PointXYZI<float> &point2);
        void GetMaxMin(PointCloud& input_point_cloud, Eigen::Vector3f& min_p, Eigen::Vector3f& max_p);

        float voxel_x_;
        float voxel_y_;
        float voxel_z_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_VOXEL_FILTER_H_
