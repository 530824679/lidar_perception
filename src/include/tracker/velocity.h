//
// Created by linuxidc on 2020/4/14.
//

#include <queue>
#include <iostream>

#include "common/utils/types.h"

namespace lidar_perception_ros{

    class Velocity{
    public:
        Velocity();
        ~Velocity();

        TrackerInfo GetTrackVelocity();
        TrackerInfo AssignVelocityValue(TrackerInfo tracker_velocity_);

        float CalculateVelocity(std::queue<float>& velocity,float now_velocity,float& sum_velocity);
    public:
        
        float z;
        float height;
        std::queue<float> velocity_x_;
        std::queue<float> velocity_y_;
        TrackerInfo tracker_velocity_;
    };

}

