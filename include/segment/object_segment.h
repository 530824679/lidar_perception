//
/******************************************************************************/
/*!
File name: object_segment.h

Description:
This file define class of Segment to transform point cloud to grid map.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_OBJECT_SEGMENT_H_
#define _LIDAR_PERCEPTION_ROS_OBJECT_SEGMENT_H_

// system include
#include <memory>

// json include
#include "common/json/json.h"

// math include
#include <math.h>
#include <unordered_set>

// local include
#include "detect/grid.h"

#include "manager/config_manager.h"
#include "common/utils/point3d.h"
#include "common/utils/types.h"
#include "Eigen/Dense"
#include "Eigen/Core"

namespace lidar_perception_ros{

    class Segment {
    public:
        struct Point3D{
            float x;
            float y;
            float z;
        };

    protected:
        typedef typename std::vector<int> Indices;
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        Segment() = default;
        Segment(SegmentParam segment_param, ROIParam roi_param);
        ~Segment();

        void ObjectSegment(const PointCloud& input_cloud, PointCloud& out_cloud, Eigen::Vector4d& plane_coefficients);
        bool Ransac3D(const PointCloud& input_cloud, Eigen::Vector4d& plane_coefficients);

        bool BuildGridMap(const PointCloud& input_cloud, PointCloud& out_cloud, Eigen::Vector4d plane_coefficients);
        float Min(float x, float y);
        float Max(float x, float y);
        void CreateRotateMatrix(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Matrix3f &rotate_matrix);
        void CoordTransfor(const PointCloud& input_cloud, Eigen::Matrix3f &rotation_matrix, PointCloud& out_cloud);
        void CalcRotateMatrix(Eigen::Vector4d plane_coefficients, Eigen::Matrix3f &rotate_matrix);
    private:
        float Distance(Point3D& pt1, Point3D& pt2);
        bool CollineationJudge(Point3D& pt1, Point3D& pt2, Point3D& pt3);

        int max_iterations_;
        float distance_tolerate_;

        int row_;
        int column_;
        float grid_size_;
        float height_threshold_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_OBJECT_SEGMENT_H_
