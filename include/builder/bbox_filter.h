#ifndef _LIDAR_PERCEPTION_ROS_BBOX_FILTER_H_
#define _LIDAR_PERCEPTION_ROS_BBOX_FILTER_H_

// system include
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <string.h>

// local include
#include "common/utils/types.h"

namespace lidar_perception_ros {

    class BBoxFilter{

    public:
        BBoxFilter() = default;
        BBoxFilter(BBoxParam param);
        ~BBoxFilter();

        void ConditionFilter(std::vector<BBox> &box_list);
        void FusionBBox(std::vector<BBox> &box_list, const double threshold=0.95);

        static float GetArea(BBox box);
        static float GetVolume(BBox box);
        static bool CompareVolume(BBox box_1, BBox box_2);

        int DCompare(float x);
        Point2D Intersection(Point2D a, Point2D b, Point2D c, Point2D d);
        float IntersectionArea(const BBox & r1, const BBox & r2);
        float Cross(Point2D a, Point2D b, Point2D c);
        float CPIA(Point2D a[], Point2D b[], int na, int nb);
        float SPIA(Point2D a[], Point2D b[], int na, int nb);
        float CalcPolygonArea(Point2D p[], int n);
        float CalcCoverage(const BBox & r1, const BBox & r2);
        float CalcArea(const BBox & r);

        bool isPointInRect(BBox box, Point2D point);
    private:
        float box_min_bottom_;
        float box_min_top_;
        float box_min_area_;
        float box_max_area_;
        float box_min_volume_;
        float box_max_volume_;
        float height_threshold_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_BBOX_FILTER_H_
