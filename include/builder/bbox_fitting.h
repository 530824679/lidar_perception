
#ifndef _LIDAR_PERCEPTION_ROS_BBOX_FITTING_H_
#define _LIDAR_PERCEPTION_ROS_BBOX_FITTING_H_

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
#include "convex_hull.h"

namespace lidar_perception_ros{

    class BBoxEstimator{
    public:
        typedef Eigen::Map<Eigen::Array3f> Array3fMap;
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<int>> IndicesPtr;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        BBoxEstimator() = default;
        BBoxEstimator(BBoxParam param);
        ~BBoxEstimator();

        void Estimate(std::vector<PointCloud> &clusters, std::vector<BBox> &bboxes);

    private:
        void GetMinMax3D(const PointCloudPtr &in_cloud_ptr, PointXYZI<float>& min_point, PointXYZI<float>& max_point, PointXYZI<float>& centroid_point);

        bool AABBFitting(PointCloudPtr &in_cloud_ptr, BBox &box);

        bool HullFitting(PointCloudPtr &in_cloud_ptr, BBox &box);

    private:
        BBoxFilter bbox_filter_;
        ConvexHull convex_hull_;

        float min_distance_;
        float max_distance_;
    };
}
#endif //_LIDAR_PERCEPTION_ROS_BBOX_FITTING_H_
