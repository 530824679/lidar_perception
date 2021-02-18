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

float Velocity::CalculateVelocity(std::queue<float> &velocity, float now_velocity, float &sum_velocity,int kernel_size) {

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

    float size=static_cast<float>(velocity.size());
    if(size<kernel_size+1){
        tmp_velocity=sum_velocity/size;
    }else{
        sum_velocity-=velocity.front();
        tmp_velocity=sum_velocity/static_cast<float>(kernel_size);
        velocity.pop();
    }
    return tmp_velocity;
}

float Velocity::CalculateMeadiumVelocity(std::deque<float> &velocity, float now_velocity,int kernel_size) {

    velocity.push_back(now_velocity);
    int size = velocity.size();

    if (size < kernel_size + 1) {
        if (size % 2 == 0) {
            std::vector<float> tmp_velocity;
            for (int i = 0; i < velocity.size(); i++) {
                tmp_velocity.push_back(velocity[i]);
            }
            std::sort(tmp_velocity.begin(), tmp_velocity.end());
            int subscript_left = static_cast<int>(size / 2.0) - 1;
            int subscript_right = static_cast<int>(size / 2.0);
            float cal_velocity = (tmp_velocity[subscript_left] + tmp_velocity[subscript_right]) / 2.0;
            return cal_velocity;
        } else {

            std::vector<float> tmp_velocity;
            for (int i = 0; i < velocity.size(); i++) {
                tmp_velocity.push_back(velocity[i]);
            }
            std::sort(tmp_velocity.begin(), tmp_velocity.end());

            int subscript = static_cast<int>(size / 2.0);
            float cal_velocity = tmp_velocity[subscript];
            return cal_velocity;
        }
    } else {
        velocity.pop_front();
        std::vector<float> tmp_velocity;
        for (int i = 0; i < velocity.size(); i++) {
            tmp_velocity.push_back(velocity[i]);
        }
        std::sort(tmp_velocity.begin(), tmp_velocity.end());
        int subscript = static_cast<int>(size / 2.0);
        return tmp_velocity[subscript];
    }
}

}

