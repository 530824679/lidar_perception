#include <iostream>
#include "segment/object_cluster.h"

namespace lidar_perception_ros {

    Cluster::Cluster(ClusterParam cluster_param)
    {
        min_cluster_size_ = cluster_param.min_cluster_size_;
        max_cluster_size_ = cluster_param.max_cluster_size_;
        cluster_distance_ = cluster_param.cluster_distance_;
        dbscan_min_points_ = cluster_param.dbscan_min_points_;
        dbscan_tolerance_ = cluster_param.dbscan_tolerance_;
        use_cluster_merge_ = cluster_param.use_cluster_merge_;
        cluster_merge_threshold_ = cluster_param.cluster_merge_threshold_;
        cluster_merge_threshold_y_ = cluster_param.cluster_merge_threshold_y_;
        use_multi_thresholds_ = cluster_param.use_multi_thresholds_;

        for(size_t i = 0; i < 2; i++)
        {
            cluster_ranges_[i] = cluster_param.cluster_ranges_[i];
        }
        for(size_t i = 0; i < 3; i++)
        {
            cluster_distances_[i] = cluster_param.cluster_distances_[i];
        }
        for(size_t i = 0; i < 3; i++)
        {
            dbscan_tolerances_[i] = cluster_param.dbscan_tolerances_[i];
        }
    }

    void Cluster::ClusterHelper(int index, PointCloud &input_cloud, Indices &cluster, std::vector<bool> &is_processed,
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

    void Cluster::EuclidCluster(PointCloud &input_cloud, float clustering_distance, std::vector<PointCloud> &cluster_cloud_vec)
    {
        if (input_cloud.size() > 0)
        {
            KdTree *tree = new KdTree;

            PointCloud cloud_2d;
            for (size_t index = 0; index < input_cloud.size(); index++) {
                PointXYZI<float> pt;
                pt.SetX(input_cloud[index].GetX());
                pt.SetY(input_cloud[index].GetY());
                cloud_2d.push_back(pt);
                tree->Insert(input_cloud[index], index);
            }

            std::vector<bool> is_processed(cloud_2d.size(), false);

            for (int index = 0; index < cloud_2d.size(); index++) {
                if (is_processed[index]) {
                    index++;
                    continue;
                }

                Indices cluster_index;
                PointCloud cloud_cluster;
                ClusterHelper(index, cloud_2d, cluster_index, is_processed, tree, clustering_distance);

                int cluster_size = cluster_index.size();
                if(cluster_size >= min_cluster_size_ && cluster_size <= max_cluster_size_)
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

    void Cluster::EuclidClusterWithMerge(PointCloud &input_cloud, std::vector<PointCloud> &cluster_cloud_vec)
    {
        std::vector<PointCloud> clusters_mid;
        if (use_multi_thresholds_)
        {
            std::vector<PointCloud> cloud_segments_array(3);
            for (size_t i = 0; i < cloud_segments_array.size(); i++)
            {
                PointCloud tmp_cloud;
                cloud_segments_array[i] = tmp_cloud;
            }

            for (size_t i = 0; i < input_cloud.size(); i++)
            {
                PointXYZI<float> current_point;
                current_point = input_cloud[i];

                // float origin_distance = sqrt(pow(current_point.GetX(), 2) + pow(current_point.GetY(), 2));
                float origin_distance = current_point.GetX();

                if (origin_distance < cluster_ranges_[0])
                {
                    cloud_segments_array[0].push_back(current_point);
                }
                else if (origin_distance < cluster_ranges_[1])
                {
                    cloud_segments_array[1].push_back(current_point);
                }
                else
                {
                    cloud_segments_array[2].push_back(current_point);
                }
            }

            for (size_t i = 0; i < cloud_segments_array.size(); i++)
            {
                EuclidCluster(cloud_segments_array[i], cluster_distances_[i], clusters_mid);
            }
            // std::cout << "clusters_mid size:" << clusters_mid.size() << std::endl;
        }
        else
        {
            EuclidCluster(input_cloud, cluster_distance_, clusters_mid);
            // std::cout << "clusters_mid size:" << clusters_mid.size() << std::endl;
        }

        if (clusters_mid.size() > 0)
        {
            if (use_cluster_merge_)
            {
                std::vector<ClusteringPtr> all_clusters;
                for (size_t i = 0; i < clusters_mid.size(); i++)
                {
                    ClusteringPtr cluster_i(new Clustering());
                    cluster_i->SetCloud(clusters_mid[i], i, true);
                    all_clusters.push_back(cluster_i);
                }

                std::vector<ClusteringPtr> final_clusters;
                checkAllForMerge(all_clusters, final_clusters);

                for (size_t i = 0; i < all_clusters.size(); i++)
                {
                    PointCloud cloud_cluster;
                    for(size_t m = 0; m < final_clusters.size(); m++)
                    {
                        if (final_clusters[m]->GetId() == i)
                        {
                            PointCloud cloud_cluster_m;
                            final_clusters[m]->GetCloud(cloud_cluster_m);
                            for (size_t j = 0; j < cloud_cluster_m.size(); j++)
                            {
                                cloud_cluster.push_back(cloud_cluster_m[j]);
                            }
                        }
                    }

                    if (cloud_cluster.size() > 0)
                    {
                        cluster_cloud_vec.push_back(cloud_cluster);
                    }
                }
                // std::cout << "cluster_cloud_vec size:" << cluster_cloud_vec.size() << std::endl;
            }
            else
            {
                cluster_cloud_vec = clusters_mid;
            }
        }
    }

    void Cluster::DBSCANCluster(PointCloud &input_cloud, float clustering_tolerance, std::vector<PointCloud> &cluster_cloud_vec)
    {
        if (input_cloud.size() > 0)
        {
            KdTree *tree = new KdTree;

            PointCloudPtr cloud_2d(new PointCloud);
            for (size_t index = 0; index < input_cloud.size(); index++)
            {
                PointXYZI<float> pt;
                pt.SetX(input_cloud[index].GetX());
                pt.SetY(input_cloud[index].GetY());
                pt.SetZ(0);
                (*cloud_2d).push_back(pt);

                tree->Insert(input_cloud[index], index);
            }

            std::vector<Indices> dbscan_indices;
            DBSCANKdtreeCluster dbscan;
            dbscan.setInputCloud(cloud_2d);
            dbscan.setCorePointMinPts(dbscan_min_points_);
            dbscan.setClusterTolerance(clustering_tolerance);
            dbscan.setMinClusterSize(min_cluster_size_);
            dbscan.setMaxClusterSize(max_cluster_size_);
            dbscan.setSearchMethod(tree);
            dbscan.extract(dbscan_indices);

            for (size_t i = 0; i < dbscan_indices.size(); i++)
            {
                PointCloud cloud_cluster;
                for (auto pit = dbscan_indices[i].begin(); pit != dbscan_indices[i].end(); ++pit)
                {
                    cloud_cluster.push_back(input_cloud[*pit]);
                }
                cluster_cloud_vec.push_back(cloud_cluster);
            }
            delete tree;
        }
    }

    void Cluster::DBSCANClusterWithMerge(PointCloud &input_cloud, std::vector<PointCloud> &cluster_cloud_vec)
    {
        // std::cout << "input_cloud to clustering size:" << input_cloud.size() << std::endl;
        std::vector<PointCloud> clusters_mid;
        if (use_multi_thresholds_)
        {
            std::vector<PointCloud> cloud_segments_array(3);
            for (size_t i = 0; i < cloud_segments_array.size(); i++)
            {
                PointCloud tmp_cloud;
                cloud_segments_array[i] = tmp_cloud;
            }

            for (size_t i = 0; i < input_cloud.size(); i++)
            {
                PointXYZI<float> current_point;
                current_point = input_cloud[i];

                // float origin_distance = sqrt(pow(current_point.GetX(), 2) + pow(current_point.GetY(), 2));
                float origin_distance = current_point.GetX();

                if (origin_distance < cluster_ranges_[0])
                {
                    cloud_segments_array[0].push_back(current_point);
                }
                else if (origin_distance < cluster_ranges_[1])
                {
                    cloud_segments_array[1].push_back(current_point);
                }
                else
                {
                    cloud_segments_array[2].push_back(current_point);
                }
            }

            for (size_t i = 0; i < cloud_segments_array.size(); i++)
            {
                DBSCANCluster(cloud_segments_array[i], dbscan_tolerances_[i], clusters_mid);
            }
            // std::cout << "clusters_mid size:" << clusters_mid.size() << std::endl;
        }
        else
        {
            DBSCANCluster(input_cloud, dbscan_tolerance_, clusters_mid);
            // std::cout << "clusters_mid size:" << clusters_mid.size() << std::endl;
        }

        if (clusters_mid.size() > 0)
        {
            if (use_cluster_merge_)
            {
                std::vector<ClusteringPtr> all_clusters;
                for (size_t i = 0; i < clusters_mid.size(); i++)
                {
                    ClusteringPtr cluster_i(new Clustering());
                    cluster_i->SetCloud(clusters_mid[i], i, true);
                    all_clusters.push_back(cluster_i);
                }

                std::vector<ClusteringPtr> final_clusters;
                checkAllForMerge(all_clusters, final_clusters);

                for (size_t i = 0; i < all_clusters.size(); i++)
                {
                    PointCloud cloud_cluster;
                    for(size_t m = 0; m < final_clusters.size(); m++)
                    {
                        if (final_clusters[m]->GetId() == i)
                        {
                            PointCloud cloud_cluster_m;
                            final_clusters[m]->GetCloud(cloud_cluster_m);
                            for (size_t j = 0; j < cloud_cluster_m.size(); j++)
                            {
                                cloud_cluster.push_back(cloud_cluster_m[j]);
                            }
                        }
                    }

                    if (cloud_cluster.size() > 0)
                    {
                        cluster_cloud_vec.push_back(cloud_cluster);
                    }
                }
                // std::cout << "cluster_cloud_vec size:" << cluster_cloud_vec.size() << std::endl;
            }
            else
            {
                cluster_cloud_vec = clusters_mid;
            }
        }
    }

    void Cluster::checkClusterMerge(size_t in_cluster_id, std::vector<ClusteringPtr> &in_clusters,
                                    std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices)
    {
        PointXYZI<float> point_a, min_point_a, max_point_a;
        in_clusters[in_cluster_id]->GetCentroid(point_a);
        in_clusters[in_cluster_id]->GetMinPoint(min_point_a);
        in_clusters[in_cluster_id]->GetMaxPoint(max_point_a);
        for (size_t i = 0; i < in_clusters.size(); i++)
        {
            if (i != in_cluster_id && !in_out_visited_clusters[i])
            {
                PointXYZI<float> point_b, min_point_b, max_point_b;
                in_clusters[i]->GetCentroid(point_b);
                in_clusters[i]->GetMinPoint(min_point_b);
                in_clusters[i]->GetMaxPoint(max_point_b);
                double distance = sqrt(pow(point_b.GetX() - point_a.GetX(), 2) + pow(point_b.GetY() - point_a.GetY(), 2));
                double distance_y = abs(point_a.GetY() - point_b.GetY());

                if (distance <= cluster_merge_threshold_ && distance_y <= cluster_merge_threshold_y_)
                {
                    in_out_visited_clusters[i] = true;
                    out_merge_indices.push_back(i);
                    // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
                    checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices);
                }
                else if (use_multi_thresholds_)
                {
                    if (((abs(min_point_a.GetX() -15) < 0.5 && abs(max_point_b.GetX() -15) < 0.5) ||
                         (abs(max_point_a.GetX() -15) < 0.5 && abs(min_point_b.GetX() -15) < 0.5)) && 
                         (min_point_a.GetY() < max_point_b.GetY() && max_point_a.GetY() > min_point_b.GetY()))
                    {
                        in_out_visited_clusters[i] = true;
                        out_merge_indices.push_back(i);
                        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
                        checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices);
                    }
                    else if (((abs(min_point_a.GetX() -30) < 0.5 && abs(max_point_b.GetX() -30) < 0.5) ||
                              (abs(max_point_a.GetX() -30) < 0.5 && abs(min_point_b.GetX() -30) < 0.5)) &&
                              (min_point_a.GetY() < max_point_b.GetY() && max_point_a.GetY() > min_point_b.GetY()))
                    {
                        in_out_visited_clusters[i] = true;
                        out_merge_indices.push_back(i);
                        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
                        checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices);
                    }
                }
            }
        }
    }

    void Cluster::mergeClusters(const std::vector<ClusteringPtr> &in_clusters, std::vector<ClusteringPtr> &out_clusters,
                                std::vector<size_t> &in_merge_indices, const size_t &current_index,
                                std::vector<bool> &in_out_merged_clusters)
    {
        PointCloud sum_cloud;
        PointCloud mono_cloud;
        ClusteringPtr merged_cluster(new Clustering());
        for (size_t i = 0; i < in_merge_indices.size(); i++)
        {
            PointCloud temp_cloud;
            in_clusters[in_merge_indices[i]]->GetCloud(temp_cloud);
            for (size_t j = 0; j < temp_cloud.size(); j++)
            {
                sum_cloud.push_back(temp_cloud[j]);
            }
            in_out_merged_clusters[in_merge_indices[i]] = true;
        }

        if (sum_cloud.size() > 0)
        {
            mono_cloud = sum_cloud;
            merged_cluster->SetCloud(mono_cloud, current_index, false);
            out_clusters.push_back(merged_cluster);
        }
    }

    void Cluster::checkAllForMerge(std::vector<ClusteringPtr> &in_clusters, std::vector<ClusteringPtr> &out_clusters)
    {
        std::vector<bool> visited_clusters(in_clusters.size(), false);
        std::vector<bool> merged_clusters(in_clusters.size(), false);
        size_t current_index = 0;
        for (size_t i = 0; i < in_clusters.size(); i++)
        {
            if (!visited_clusters[i])
            {
                visited_clusters[i] = true;
                std::vector<size_t> merge_indices;
                checkClusterMerge(i, in_clusters, visited_clusters, merge_indices);
                mergeClusters(in_clusters, out_clusters, merge_indices, i, merged_clusters);
            }
        }
        for (size_t i = 0; i < in_clusters.size(); i++)
        {
            // check for clusters not merged, add them to the output
            if (!merged_clusters[i])
            {
                out_clusters.push_back(in_clusters[i]);
            }
        }
    }
}
