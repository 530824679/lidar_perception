#ifndef _LIDAR_PERCEPTION_ROS_BBOX_FILTER_H_
#define _LIDAR_PERCEPTION_ROS_BBOX_FILTER_H_

// system include
#include <iostream>
#include <vector>

// local include
#include "common/utils/types.h"

namespace lidar_perception_ros {

    class BBoxFilter{

    public:
        BBoxFilter() = default;
        BBoxFilter(BBoxParam param);
        ~BBoxFilter();

        void ConditionFilter(std::vector<BBox> &box_list);
        float GetVolume(BBox box);

    private:
        float box_min_bottom_;
        float box_min_top_;
        float box_min_volume_;
        float box_max_volume_;
        float height_threshold_;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_BBOX_FILTER_H_
