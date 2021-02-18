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

// system include
#include <memory>

// local include
#include "common/utils/point3d.h"
#include "common/utils/kdtree3d.h"
#include "common/utils/types.h"
#include "segment/DBSCAN_kdtree.h"
#include "segment/clustering.h"

namespace lidar_perception_ros{

    class Cluster{
    protected:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;
        typedef typename std::vector<int> Indices;

    public:
        Cluster() = default;
        Cluster(ClusterParam cluster_param);
        ~Cluster(){};

        void ClusterHelper(int index, PointCloud &input_cloud, Indices &cluster, std::vector<bool> &is_processed, KdTree *tree, float cluster_distance);
        void EuclidCluster(PointCloud &input_cloud, float clustering_distance, std::vector<PointCloud> &cluster_cloud_vec);
        void EuclidClusterWithMerge(PointCloud &input_cloud, std::vector<PointCloud> &cluster_cloud_vec);
        void DBSCANCluster(PointCloud &input_cloud, float clustering_tolerance, std::vector<PointCloud> &cluster_cloud_vec);
        void DBSCANClusterWithMerge(PointCloud &input_cloud, std::vector<PointCloud> &cluster_cloud_vec);

        void checkClusterMerge(size_t in_cluster_id, std::vector<ClusteringPtr> &in_clusters, std::vector<bool> &in_out_visited_clusters,
                               std::vector<size_t> &out_merge_indices);
        void mergeClusters(const std::vector<ClusteringPtr> &in_clusters, std::vector<ClusteringPtr> &out_clusters,
                           std::vector<size_t> &in_merge_indices, const size_t &current_index, std::vector<bool> &in_out_merged_clusters);
        void checkAllForMerge(std::vector<ClusteringPtr> &in_clusters, std::vector<ClusteringPtr> &out_clusters);

    private:
        int num_points_;
        int min_cluster_size_;
        int max_cluster_size_;
        float cluster_distance_;
        int dbscan_min_points_;
        float dbscan_tolerance_;
        bool use_cluster_merge_;
        float cluster_merge_threshold_;
        float cluster_merge_threshold_y_;
        bool use_multi_thresholds_;
        float cluster_ranges_[2];
        float cluster_distances_[3];
        float dbscan_tolerances_[3];
    };
}

#endif //_LIDAR_PERCEPTION_ROS_OBJECT_CLUSTER_H_
