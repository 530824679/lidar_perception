#include "detect/grid.h"

namespace lidar_perception_ros
{
    Grid::Grid()
    {
        init_ = false;
        type_ = UNKNOW;
        points_num_ = 0;
        min_height_ = 0.0;
        max_height_ = 0.0;
        height_diff_ = 0.0;
        intensity_ave_ = 0.0;
        is_roadside_ = false;
    }

    Grid::~Grid() {

    }
}

