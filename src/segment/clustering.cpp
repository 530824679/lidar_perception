#include "segment/clustering.h"

namespace lidar_perception_ros
{
Clustering::Clustering()
{
}

void Clustering::GetBoundingBox(BBox& bounding_box)
{
    bounding_box = bounding_box_;
}

void Clustering::GetCloud(PointCloud& clustering_cloud)
{
    clustering_cloud = pointcloud_;
}

void Clustering::GetMinPoint(PointXYZI<float>& min_point)
{
    min_point = min_point_;
}

void Clustering::GetMaxPoint(PointXYZI<float>& max_point)
{
    max_point = max_point_;
}

void Clustering::GetCentroid(PointXYZI<float>& centroid)
{
    centroid = centroid_;
}

void Clustering::SetCloud(const PointCloud& in_origin_cloud, int in_id, bool in_estimate_pose)
{
    id_ = in_id;
    pointcloud_ = in_origin_cloud;

    // extract pointcloud using the indices
    // calculate min and max points
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < in_origin_cloud.size(); i++)
    {
        centroid_.SetX(centroid_.GetX() + in_origin_cloud[i].GetX());
        centroid_.SetY(centroid_.GetY() + in_origin_cloud[i].GetY());
        centroid_.SetZ(centroid_.GetZ() + in_origin_cloud[i].GetZ());

        if (in_origin_cloud[i].GetX() < min_x)
            min_x = in_origin_cloud[i].GetX();
        if (in_origin_cloud[i].GetY() < min_y)
            min_y = in_origin_cloud[i].GetY();
        if (in_origin_cloud[i].GetZ() < min_z)
            min_z = in_origin_cloud[i].GetZ();
        if (in_origin_cloud[i].GetX() > max_x)
            max_x = in_origin_cloud[i].GetX();
        if (in_origin_cloud[i].GetY() > max_y)
            max_y = in_origin_cloud[i].GetY();
        if (in_origin_cloud[i].GetZ() > max_z)
            max_z = in_origin_cloud[i].GetZ();
    }
    // min, max points
    min_point_.SetX(min_x);
    min_point_.SetY(min_y);
    min_point_.SetZ(min_z);
    max_point_.SetX(max_x);
    max_point_.SetY(max_y);
    max_point_.SetZ(max_z);

    // calculate centroid, average
    if (in_origin_cloud.size() > 0)
    {
        centroid_.SetX(centroid_.GetX() / in_origin_cloud.size());
        centroid_.SetY(centroid_.GetY() / in_origin_cloud.size());
        centroid_.SetZ(centroid_.GetZ() / in_origin_cloud.size());
    }

    if (in_estimate_pose)
    {
        // pose estimation
        PointCloud hull;
        PointCloudPtr points(new PointCloud(in_origin_cloud));
        convex_hull_.GrahamScan(points, hull);
        convex_hull_.MinAreaRect(hull, bounding_box_);

        bounding_box_.dz = max_point_.GetZ() - min_point_.GetZ();
        bounding_box_.z = min_point_.GetZ() + bounding_box_.dz / 2;
    }
}

int Clustering::GetId()
{
    return id_;
}

Clustering::~Clustering()
{
    // TODO Auto-generated destructor stub
}
}  // namespace lidar_perception_ros