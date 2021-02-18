#include <math.h>
#include <assert.h>
#include <algorithm>
#include "common/utils/types.h"

namespace lidar_perception_ros{
    
    class rotated_iou{
        public:
        enum RectanglesIntersectTypes {
            INTERSECT_NONE = 0, //!< No intersection
            INTERSECT_PARTIAL  = 1, //!< There is a partial intersection
            INTERSECT_FULL  = 2 //!< One of the rectangle is fully enclosed in the other
        };
        rotated_iou();
        ~rotated_iou();
        int isNaN(float value);
        int isInf(float value);
        void points(Point2D pt[]) const;
        int rotatedRectangleIntersection(const RotatedRect& rect1, const RotatedRect& rect2, std::vector<Point2D>& intersectingRegion);
        void convexHull(std::vector<Point2D> input_points,std::vector<Point2D>& output_points);
        double counterArea(std::vector<Point2D> order_points);
};

}

