#include <iostream>
#include "segment/object_cluster.h"

namespace lidar_perception_ros {

    void Cluster::ClusterHelper(int index, PointCloud input_cloud, Indices &cluster, std::vector<bool> &is_processed,
                                KdTree *tree, float cluster_distance) {
        is_processed[index] = true;
        cluster.push_back(index);

        Indices nearest_point = tree->Search(input_cloud[index], cluster_distance);
        for (int nearest_id : nearest_point) {
            if (!is_processed[nearest_id]) {
                ClusterHelper(nearest_id, input_cloud, cluster, is_processed, tree, cluster_distance);
            }
        }
    }

    void Cluster::PclCluster(PointCloud input_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>> &cluster_cloud_vec, ClusterParam cluster_param)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < input_cloud.size(); i++)
        {
            pcl::PointXYZ pcl_pt;
            pcl_pt.x = input_cloud[i].GetX();
            pcl_pt.y = input_cloud[i].GetY();
            pcl_pt.z = 0;
            cloud_2d->points.push_back(pcl_pt);
        }

        std::cout << "cloud 2d size is: " << cloud_2d->points.size() << std::endl;
        if (cloud_2d->points.size() > 0)
            tree->setInputCloud(cloud_2d);

        std::vector<pcl::PointIndices> local_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean;
        euclidean.setInputCloud(cloud_2d);
        euclidean.setClusterTolerance(cluster_param.cluster_distance_);
        euclidean.setMinClusterSize(cluster_param.min_cluster_size_);
        euclidean.setMaxClusterSize(cluster_param.max_cluster_size_);
        euclidean.setSearchMethod(tree);
        euclidean.extract(local_indices);

        for (size_t i = 0; i < local_indices.size(); i++){
            pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
            for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit) {
                pcl::PointXYZ pt;
                pt.x = input_cloud[*pit].GetX();
                pt.y = input_cloud[*pit].GetY();
                pt.z = input_cloud[*pit].GetZ();
                cloud_cluster.push_back(pt);
            }
            cluster_cloud_vec.push_back(cloud_cluster);
        }
    }

    void Cluster::PclEuclidCluster(PointCloud input_cloud, std::vector<PointCloud> &cluster_cloud_vec, ClusterParam cluster_param)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < input_cloud.size(); i++)
        {
            pcl::PointXYZ pcl_pt;
            pcl_pt.x = input_cloud[i].GetX();
            pcl_pt.y = input_cloud[i].GetY();
            pcl_pt.z = 0;
            cloud_2d->points.push_back(pcl_pt);
        }

        std::cout << "cloud 2d size is: " << cloud_2d->points.size() << std::endl;
        if (cloud_2d->points.size() > 0)
            tree->setInputCloud(cloud_2d);

        std::vector<pcl::PointIndices> local_indices;

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean;
        euclidean.setInputCloud(cloud_2d);
        euclidean.setClusterTolerance(cluster_param.cluster_distance_);
        euclidean.setMinClusterSize(cluster_param.min_cluster_size_);
        euclidean.setMaxClusterSize(cluster_param.max_cluster_size_);
        euclidean.setSearchMethod(tree);
        euclidean.extract(local_indices);

        for (size_t i = 0; i < local_indices.size(); i++){
            PointCloud cloud_cluster;
            for (auto pit = local_indices[i].indices.begin(); pit != local_indices[i].indices.end(); ++pit) {
                cloud_cluster.push_back(input_cloud[*pit]);
            }
            cluster_cloud_vec.push_back(cloud_cluster);
        }
    }

    void Cluster::EuclidCluster(PointCloud input_cloud, std::vector<PointCloud> &cluster_cloud_vec, ClusterParam cluster_param)
    {
        KdTree *tree = new KdTree;

        for (size_t i = 0; i < input_cloud.size(); i++) {
            input_cloud[i].SetZ(0);
        }

        for (int index = 0; index < input_cloud.size(); index++)
        {
            tree->Insert(input_cloud[index], index);
        }

        std::vector<bool> is_processed(input_cloud.size(), false);

        for (int index = 0; index < input_cloud.size(); index++) {
            if (is_processed[index]) {
                index++;
                continue;
            }

            Indices cluster_index;
            PointCloud cloud_cluster;
            ClusterHelper(index, input_cloud, cluster_index, is_processed, tree, cluster_param.cluster_distance_);

            int cluster_size = cluster_index.size();
            if(cluster_size >= cluster_param.min_cluster_size_ && cluster_size <= cluster_param.max_cluster_size_)
            {
                for (int i = 0; i < cluster_size; i++) {
                    cloud_cluster.push_back(input_cloud[cluster_index[i]]);
                }
                cluster_cloud_vec.push_back(cloud_cluster);
            }
        }
        delete tree;
    }

}
