/******************************************************************************/
/*!
File name: bbox_fitting.h

Description:
This file define class of BBoxEstimator use to fitting the bounding box of cluster bundles.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_BBOX_FITTING_L_SHAPE_H_
#define _LIDAR_PERCEPTION_ROS_BBOX_FITTING_L_SHAPE_H_

// system include
#include <math.h>
#include <cfloat>
#include <iostream>
#include <memory>

// local include
#include "Eigen/Eigen"
#include "common/utils/point3d.h"
#include "common/utils/types.h"
#include "bbox_filter.h"

namespace lidar_perception_ros{

    class LShapeBBoxEstimator{
    public:
        typedef Eigen::Map<Eigen::Array3f> Array3fMap;
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<int>> IndicesPtr;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        LShapeBBoxEstimator(BBoxParam param);
        ~LShapeBBoxEstimator();

        void Estimate(std::vector<PointCloud> &clusters, std::vector<BBox> &bboxes);

    private:
        // tmp add rect
        void GetMinMax3D(const PointCloudPtr &in_cloud_ptr, PointXYZI<float>& min_point, PointXYZI<float>& max_point, PointXYZI<float>& centroid_point);

        bool SearchBasedFitting(PointCloudPtr &in_cloud_ptr, BBox &box);

        float CalcCloseness(const std::vector<float> &C_1, const std::vector<float> &C_2);

        bool CalcBBox(PointCloudPtr &in_cloud_ptr, std::vector<std::pair<float, float>> &Q, float dz, BBox &box);

        void CalcRectPoints(BBox &box);

        void CalcCrossPoint(const float a0, const float a1, const float b0, const float b1, const float c0, const float c1, float& x, float& y);
    private:
        BBoxFilter bbox_filter_;
        std::vector<float> a_;
        std::vector<float> b_;
        std::vector<float> c_;
    };
}
#endif //_LIDAR_PERCEPTION_ROS_BBOX_FITTING_L_SHAPE_H_
