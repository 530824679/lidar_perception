/******************************************************************************/
/*!
File name: render.h

Description:
This file define class of Render use to visualize point cloud.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _PERCEPTION_ROS_LIDAR_RENDER_H_
#define _PERCEPTION_ROS_LIDAR_RENDER_H_

// pcl include
#include <pcl/visualization/pcl_visualizer.h>

// local include
#include "common/utils/point3d.h"
#include "common/utils/types.h"

namespace lidar_perception_ros {

    class Render {
    public:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        Render();

        ~Render();

        void InitCamera();

        void RenderGround();

        void RenderPointCloud(const PointCloudPtr &input_cloud_ptr, std::string name, Color color = Color(1, 1, 1));

        void RenderDoublePointCloud(const PointCloudPtr &before_cloud_ptr, const PointCloudPtr &after_cloud_ptr, std::string name, Color color);

        void RenderCluster(const std::vector<PointCloud> &input_cloud, std::string name);

        void RenderDoubleCluster(const PointCloudPtr &input_cloud_ptr, const std::vector<PointCloud> &input_cloud, std::string name, Color color);

        void RenderBBox(const std::vector<BBox> bboxes, Color color);

        void RenderDoubleBBox(const PointCloudPtr& curb_cloud_ptr, const std::vector<PointCloud>& cluster_cloud,
                              const std::vector<BBox> bboxes, std::string name, Color color);

        void RenderDLBBox(const PointCloudPtr &input_cloud_ptr, const std::vector<BBox> bboxes, std::string name, Color color);

        void KeepWait();

        void KeepOnce();

    private:
        int v1;
        int v2;
        pcl::visualization::PCLVisualizer::Ptr viewer_;
    };
}
#endif //_PERCEPTION_ROS_LIDAR_RENDER_H_