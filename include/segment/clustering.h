#ifndef CLUSTER_H_
#define CLUSTER_H_

// system include
#include <memory>

// local include
#include "builder/convex_hull.h"
#include "common/utils/point3d.h"
#include "common/utils/types.h"

namespace lidar_perception_ros
{
class Clustering
{
public:
    typedef typename std::vector<PointXYZI<float>> PointCloud;
    typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

private:
    int id_;
    PointCloud pointcloud_;
    PointXYZI<float> min_point_;
    PointXYZI<float> max_point_;
    PointXYZI<float> centroid_;
    BBox bounding_box_;

    ConvexHull convex_hull_;

public:
    /* \brief Constructor. Creates a Cluster object using the specified points in a PointCloud
   * \param[in] in_origin_cloud_ptr   Origin PointCloud of the Cluster
   * \param[in] in_id         ID of the cluster
   * \param[in] in_estimate_pose    Flag to enable Pose Estimation of the Bounding Box
   * */
    void SetCloud(const PointCloud& in_origin_cloud, int in_id, bool in_estimate_pose);

    Clustering();
    virtual ~Clustering();

    /* \brief Returns the pointer to the PointCloud containing the points in this Cluster */
    void GetCloud(PointCloud& clustering_cloud);
    /* \brief Returns the minimum point in the cluster */
    void GetMinPoint(PointXYZI<float>& min_point);
    /* \brief Returns the maximum point in the cluster*/
    void GetMaxPoint(PointXYZI<float>& max_point);
    /* \brief Returns the centroid point in the cluster */
    void GetCentroid(PointXYZI<float>& centroid);
    /* \brief Returns the calculated BoundingBox of the object */
    void GetBoundingBox(BBox& bounding_box);

    /* \brief Returns the Id of the Cluster */
    int GetId();
};

typedef std::shared_ptr<Clustering> ClusteringPtr;

}  // namespace lidar_perception_ros

#endif /* CLUSTER_H_ */