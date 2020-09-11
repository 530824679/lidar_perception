
#ifndef _LIDAR_PERCEPTION_ROS_CONVEX_HULL_H_
#define _LIDAR_PERCEPTION_ROS_CONVEX_HULL_H_

// system include
#include <iostream>
#include <cfloat>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstring>
#include <boost/shared_ptr.hpp>

// local include
#include "common/utils/point3d.h"
#include "common/utils/types.h"

namespace lidar_perception_ros {

    class ConvexHull {
    public:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<int>> IndicesPtr;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

        struct Point2D{
            float x;
            float y;
        };

    public:
        ConvexHull()= default;
        ~ConvexHull(){};

    public:
        void GrahamScan(PointCloudPtr& clusters_ptr, PointCloud& hull);
        void MinAreaRect(PointCloud &hull, BBox &box);

    private:
        float GetDistance(const Point2D p1, const Point2D p2);
        void GetAngle(PointCloud &hull, vector<float> &angle);
        void UniqueAngle(vector<float> &angle, vector<unsigned int> &state);
        void GetRotatePoints(PointCloud &hull, float theta, vector<Point2D> &rotate_points);
        void GetRemapPoints(vector<Point2D> &rotate_points, float theta, vector<Point2D> &remap_points);
    };
}

#endif //_LIDAR_PERCEPTION_ROS_CONVEX_HULL_H_
