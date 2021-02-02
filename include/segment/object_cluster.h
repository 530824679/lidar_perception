//
/******************************************************************************/
/*!
File name: object_cluster.h

Description:
This file define class of Segment to cluster object.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/


#ifndef _LIDAR_PERCEPTION_ROS_OBJECT_CLUSTER_H_
#define _LIDAR_PERCEPTION_ROS_OBJECT_CLUSTER_H_

// pcl include
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/extract_indices.h>

// system include
#include <boost/shared_ptr.hpp>

// local include
#include "common/utils/point3d.h"
#include "common/utils/kdtree3d.h"
#include "common/utils/types.h"

namespace lidar_perception_ros{

    class Cluster{
    protected:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::vector<int> Indices;

    public:
        Cluster() = default;
        ~Cluster(){};

        void ClusterHelper(int index, PointCloud input_cloud, Indices &cluster, std::vector<bool> &is_processed, KdTree *tree, float cluster_distance);
        void EuclidCluster(PointCloud input_cloud, std::vector<PointCloud> &cluster_cloud_vec, ClusterParam cluster_param);
        void PclEuclidCluster(PointCloud input_cloud, std::vector<PointCloud> &cluster_cloud_vec, ClusterParam cluster_param);
        void PclCluster(PointCloud input_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>> &cluster_cloud_vec, ClusterParam cluster_param);

    private:
        int num_points_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_OBJECT_CLUSTER_H_
