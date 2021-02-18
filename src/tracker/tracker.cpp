#include "tracker/tracker.h"

namespace lidar_perception_ros{

    Tracker::Tracker() : kf_(8, 4),coast_cycles_(0), hit_streak_(0),lidar_rate_(0),acc_threshold_(0),kf_velocity_(4,2){
        // state - center_x, center_y, length, width,yaw, v_cx, v_cy, v_length, v_width,v_yaw
    kf_.F_ <<
           1, 0, 0, 0, 0.1, 0, 0, 0,
           0, 1, 0, 0, 0, 0.1, 0, 0,
           0, 0, 1, 0, 0, 0, 0.1, 0,
           0, 0, 0, 1, 0, 0, 0, 0.1,
           0, 0, 0, 0, 1, 0, 0, 0,
           0, 0, 0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 0, 0, 1;

    // Give high uncertainty to the unobservable initial velocities
    kf_.P_ <<
           10, 0, 0, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0, 0, 0,
            0, 0, 0, 10, 0, 0, 0, 0,
            0, 0, 0, 0, 10000, 0, 0, 0,
            0, 0, 0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0, 0, 10000;


    kf_.H_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0, 0, 0,
           0, 0, 0, 1, 0, 0, 0, 0;

    kf_.Q_ <<
            0.08, 0, 0, 0, 0, 0, 0, 0,
            0, 0.08, 0, 0, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0, 0, 0,
            0, 0, 0, 0, 0.008, 0, 0, 0,
            0, 0, 0, 0, 0, 0.008, 0, 0,
            0, 0, 0, 0, 0, 0, 0.0001, 0,
            0, 0, 0, 0, 0, 0, 0, 0.0001;

    kf_.R_ <<
            1, 0, 0,  0,
            0, 1, 0,  0,
            0, 0, 10, 0,
            0, 0, 0,  10;

            
    kf_velocity_.F_ <<
           1, 0, 0.1, 0, 
           0, 1, 0, 0.1, 
           0, 0, 1, 0, 
           0, 0, 0, 1;

    // Give high uncertainty to the unobservable initial velocities
    kf_velocity_.P_ <<
           10, 0, 0, 0, 
            0, 10, 0, 0,
            0, 0, 10000, 0,
            0, 0, 0, 10000;
           

    kf_velocity_.H_ <<
           1, 0, 0, 0,
           0, 1, 0, 0;

    kf_velocity_.Q_ <<
            3, 0, 0, 0, 
            0, 3, 0, 0, 
            0, 0, 20, 0,
            0, 0, 0, 20;

    kf_velocity_.R_ <<
            5, 0, 
            0, 5; 

    }

    Tracker::~Tracker(){

    }

    void Tracker::Predict() {
        kf_.Predict();
        kf_velocity_.Predict();
        // hit streak count will be reset
        if (coast_cycles_ > 0){
            hit_streak_ = 0;
        }

        // accumulate coast cycle count
        coast_cycles_++;
    }

    void Tracker::Update(const BBox& bbox) {
        coast_cycles_ = 0;
        hit_streak_++;
        Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
        Eigen::VectorXd velocity_observation = ConvertBottomToObservation(bbox);
  
        kf_.Update(observation);
        kf_velocity_.Update(velocity_observation);
        
        bbox_.center_x = bbox.x;
        bbox_.center_y = bbox.y;
        bbox_.center_z = bbox.z;

        bbox_.width = bbox.dx;
        bbox_.length = bbox.dy;
        bbox_.height = bbox.dz;

        bbox_.centroid_x = bbox.centroid_x;
        bbox_.centroid_y = bbox.centroid_y;
        bbox_.centroid_z = bbox.centroid_z;

        bbox_.points_num = bbox.points_num;

        bbox_.angle = bbox.rotate;

        bbox_.velocity_x = kf_velocity_.x_[2];
        bbox_.velocity_y = kf_velocity_.x_[3];

        RejectOutlier(bbox_);
        
        bbox_.corners.clear();
        if(!bbox.vertex_pts.empty()){
            for(int i=0;i<4;i++){
                bbox_.corners.push_back(bbox.vertex_pts[i]);
            }
        }
        
        ++bbox_.year;
        bbox_.pre_velocity_x = bbox_.velocity_x;
        bbox_.pre_velocity_y = bbox_.velocity_y;

    }

    // Create and initialize new trackers for unmatched detections, with initial bounding box
    void Tracker::Init(const BBox &bbox) {
        kf_.x_.head(4) << ConvertBboxToObservation(bbox);
        kf_velocity_.x_.head(2) << ConvertBottomToObservation(bbox);
        bbox_.center_x = bbox.x;
        bbox_.center_y = bbox.y;
        bbox_.center_z = bbox.z;

        bbox_.width = bbox.dx;
        bbox_.length = bbox.dy;
        bbox_.height = bbox.dz;

        bbox_.centroid_x = bbox.centroid_x;
        bbox_.centroid_y = bbox.centroid_y;
        bbox_.centroid_z = bbox.centroid_z;

        bbox_.points_num = bbox.points_num;
 
        bbox_.angle = bbox.rotate;
        bbox_.corners.clear();
        
        if(!bbox.vertex_pts.empty()){
            for(int i=0;i<4;i++){
                bbox_.corners.push_back(bbox.vertex_pts[i]);

            }
        }
        bbox_.year += 1;

        hit_streak_++;
    }

    TrackerInfo Tracker::GetStateAsBbox() const{
        return ConvertStateToBbox(kf_.x_,kf_velocity_.x_);
    }

    TrackerInfo Tracker::GetStateAsInputBbox() const{
        return ConvertStateToInputBbox(bbox_);
    }

    float Tracker::GetNIS() const {
        return kf_.NIS_;
    }

    Eigen::VectorXd Tracker::ConvertBboxToObservation(const BBox& bbox) const{
        Eigen::VectorXd observation = Eigen::VectorXd::Zero(4);


        auto length = static_cast<float>(bbox.rect_dx);
        auto width = static_cast<float>(bbox.rect_dy);

        float center_x = bbox.rect_x;
        float center_y = bbox.rect_y;


        observation << center_x, center_y, length, width;
        return observation;
    }

    Point2D Tracker::NearestBboxCorner(const BBox& bbox)const{
        Point2D nearest_new_bbox_corner = bbox.vertex_pts[0];
        float min_corner_distance = sqrt(bbox.vertex_pts[0].x*bbox.vertex_pts[0].x + bbox.vertex_pts[0].y*bbox.vertex_pts[0].y);
         for (size_t i = 1; i < 4; ++i) {
             float corner_distance = sqrt(bbox.vertex_pts[i].x*bbox.vertex_pts[i].x + bbox.vertex_pts[i].y*bbox.vertex_pts[i].y);
             if(corner_distance < min_corner_distance){
                 min_corner_distance = corner_distance;
                 nearest_new_bbox_corner = bbox.vertex_pts[i];
             }
         }

         return nearest_new_bbox_corner;

    }


    Eigen::VectorXd Tracker::ConvertBottomToObservation(const BBox& bbox) const{
        Eigen::VectorXd observation = Eigen::VectorXd::Zero(2);

        float bottom_center_x = bbox.rect_x-bbox.rect_dx/2;
        float bottom_center_y = bbox.rect_dy;

        Point2D nearest_corner = NearestBboxCorner(bbox);
        //observation << nearest_corner.x, nearest_corner.y;

        observation << bottom_center_x, bottom_center_y;

        return observation;
    }


    TrackerInfo Tracker::ConvertStateToBbox(const Eigen::VectorXd &state,const Eigen::VectorXd &velocity_state) const{
        TrackerInfo tracker;
        tracker.center_x = state[0];
        tracker.center_y = state[1];

        tracker.width = state[2];
        tracker.length = state[3];

        //box.velocity_x = state[4];
        //box.velocity_y = state[5];

        // box.centroid_x = velocity_state[0];
        // box.centroid_y = velocity_state[1];

        tracker.velocity_x = velocity_state[2];
        tracker.velocity_y = velocity_state[3];

        // std::cout<<"velocity_state[2]"<<velocity_state[2]<<std::endl;


        return tracker;
    }

    int Tracker::GetCoastCycles(){
        return coast_cycles_;
    }

    int Tracker::GetHitStreak(){
        return hit_streak_;
    }

    bool Tracker::RejectOutlier(TrackerInfo& bbox_){
        // if(bbox_.year>1){
            float acceleration_x=(bbox_.velocity_x-bbox_.pre_velocity_x)*lidar_rate_;
            float acceleration_y=(bbox_.velocity_y-bbox_.pre_velocity_y)*lidar_rate_;

            if((acceleration_x>acc_threshold_)&&(acceleration_y>acc_threshold_)){
                bbox_.velocity_x=0;
                bbox_.velocity_y=0;
                return true;
            }
        // }
        return false;
    }

    TrackerInfo Tracker::ConvertStateToInputBbox(const TrackerInfo& bbox)const{
        return bbox;
    }

}


