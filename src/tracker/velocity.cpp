//
// Created by linuxidc on 2020/4/29.
//

#include "tracker/velocity.h"

namespace lidar_perception_ros{

    Velocity::Velocity(){
        tracker_velocity_.center_x=0;
        tracker_velocity_.center_y=0;
        tracker_velocity_.pre_center_x=0;
        tracker_velocity_.pre_center_y=0;
        tracker_velocity_.pre_velocity_x=0;
        tracker_velocity_.pre_velocity_y=0;
        tracker_velocity_.velocity_x=0;
        tracker_velocity_.velocity_y=0;
        tracker_velocity_.sum_velocity_x=0;
        tracker_velocity_.sum_velocity_y=0;
        tracker_velocity_.year=0;
    }

    Velocity::~Velocity(){};

    TrackerInfo Velocity::GetTrackVelocity(){
        return tracker_velocity_;
    }

    float Velocity::CalculateVelocity(std::queue<float> &velocity, float now_velocity, float &sum_velocity) {

        float tmp_velocity=0;

        if((tmp_velocity>100)&&(tmp_velocity<-100)){
            printf("The velocity is too large!\n");
            return 0;
        }

        velocity.push(now_velocity);
        sum_velocity+=now_velocity;

        if((sum_velocity>500)&&(sum_velocity<-500)){
            printf("The velocity is too large!\n");
            return 0;
        }

        int size=velocity.size();
        if(size<6){
            tmp_velocity=sum_velocity/size;
        }else{
            sum_velocity-=velocity.front();
            tmp_velocity=sum_velocity/5;
            velocity.pop();
        }
        return tmp_velocity;
    }
}
