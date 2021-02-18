#ifndef DBSCAN_KDTREE_H
#define DBSCAN_KDTREE_H

#include <algorithm>
#include <limits>

#include "common/utils/kdtree3d.h"
#include "common/utils/point3d.h"

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2

inline bool comparePointClusters (const std::vector<int> &a, const std::vector<int> &b) {
    return (a.size () < b.size ());
}

namespace lidar_perception_ros
{
class DBSCANKdtreeCluster
{
public:
    typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;
    typedef typename std::vector<int> Indices;

    virtual void setInputCloud(PointCloudPtr cloud)
    {
        input_cloud_ = cloud;
    }

    void setSearchMethod(KdTree *tree)
    {
        search_method_ = tree;
    }

    void extract(std::vector<Indices> &cluster_indices)
    {
        std::vector<int> nn_indices;
        std::vector<bool> is_noise((*input_cloud_).size(), false);
        std::vector<int> types((*input_cloud_).size(), UN_PROCESSED);
        for (int i = 0; i < (*input_cloud_).size(); i++)
        {
            if (types[i] == PROCESSED)
            {
                continue;
            }
            int nn_size = search_method_->radiusSearch((*input_cloud_)[i], eps_, nn_indices);
            if (nn_size < minPts_)
            {
                is_noise[i] = true;
                continue;
            }

            std::vector<int> seed_queue;
            seed_queue.push_back(i);
            types[i] = PROCESSED;

            for (int j = 0; j < nn_size; j++)
            {
                if (nn_indices[j] != i)
                {
                    seed_queue.push_back(nn_indices[j]);
                    types[nn_indices[j]] = PROCESSING;
                }
            }  // for every point near the chosen core point.
            int sq_idx = 1;
            while (sq_idx < seed_queue.size())
            {
                int cloud_index = seed_queue[sq_idx];
                if (is_noise[cloud_index] || types[cloud_index] == PROCESSED)
                {
                    // seed_queue.push_back(cloud_index);
                    types[cloud_index] = PROCESSED;
                    sq_idx++;
                    continue;  // no need to check neighbors.
                }
                nn_size = search_method_->radiusSearch((*input_cloud_)[cloud_index], eps_, nn_indices);
                if (nn_size >= minPts_)
                {
                    for (int j = 0; j < nn_size; j++)
                    {
                        if (types[nn_indices[j]] == UN_PROCESSED)
                        {
                            seed_queue.push_back(nn_indices[j]);
                            types[nn_indices[j]] = PROCESSING;
                        }
                    }
                }

                types[cloud_index] = PROCESSED;
                sq_idx++;
            }
            if (seed_queue.size() >= min_pts_per_cluster_ && seed_queue.size() <= max_pts_per_cluster_)
            {
                Indices r;
                r.resize(seed_queue.size());
                for (int j = 0; j < seed_queue.size(); ++j)
                {
                    r[j] = seed_queue[j];
                }
                // These two lines should not be needed: (can anyone confirm?) -FF
                std::sort(r.begin(), r.end());
                r.erase(std::unique(r.begin(), r.end()), r.end());

                cluster_indices.push_back(r);  // We could avoid a copy by working directly in the vector
            }
        }  // for every point in input cloud
        std::sort(cluster_indices.rbegin(), cluster_indices.rend(), comparePointClusters);
    }

    void setClusterTolerance(double tolerance)
    {
        eps_ = tolerance;
    }

    void setMinClusterSize(int min_cluster_size)
    {
        min_pts_per_cluster_ = min_cluster_size;
    }

    void setMaxClusterSize(int max_cluster_size)
    {
        max_pts_per_cluster_ = max_cluster_size;
    }

    void setCorePointMinPts(int core_point_min_pts)
    {
        minPts_ = core_point_min_pts;
    }

protected:
    PointCloudPtr input_cloud_;
    KdTree *search_method_;

    double eps_{0.0};
    int minPts_{1};  // not including the point itself.
    int min_pts_per_cluster_{1};
    int max_pts_per_cluster_{std::numeric_limits<int>::max()};
};  // class DBSCANCluster
}  // namespace lidar_perception_ros

#endif  // DBSCAN_KDTREE_H