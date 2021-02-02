#include "tracker/tracking.h"

namespace lidar_perception_ros{

    Tracking::Tracking(TrackerParam param) {
        frame_index_ = 1;
        current_id_ = 1;

        max_coast_cycles_ = param.max_coast_cycles_;
        min_hits_ = param.min_hits_;
        lidar_rate_ = param.lidar_rate_;
        acceleration_threshold_ = param.acceleration_threshold_;
        min_confidence_ = param.min_confidence_;
        filter_threshold_ = param.filter_threshold_;
        
    }

    Tracking::~Tracking() {

    }

    void Tracking::Process(std::vector<BBox> bboxes, lidar_perception::ObjectInfoArray& object_info_msg)
    {
        Track(tracks_, bboxes,frame_index_, current_id_, object_info_msg);
        frame_index_++;
    }

    float Tracking::CalculateIou(const BBox& det, const Tracker& track) {
        auto trk = track.GetStateAsBbox();

        // auto xx1 = std::min((det.x + det.dx/2), (trk.x + trk.dx/2));
        // auto yy1 = std::min((det.y + det.dy/2), (trk.y + trk.dy/2));
        // auto xx2 = std::max((det.x - det.dx/2), (trk.x - det.dx/2));
        // auto yy2 = std::max((det.y - det.dy/2), (trk.y - det.dy/2));

        // // std::cout<<"x1:"<<det.x + det.dx/2<<" y1:"<<det.y + det.dy/2<<" x2:"<<det.x - det.dx/2<<" y2"<<det.y - det.dy/2<<std::endl;
        // // std::cout<<"tx1:"<<trk.x + trk.dx/2<<" ty1:"<<trk.y + trk.dy/2<<" tx2:"<<trk.x - det.dx/2<<" ty2"<<trk.y - det.dy/2<<std::endl;

        // // std::cout<<"x1:"<<det.x <<" y1:"<<det.y <<" dx"<<det.dx<<" dy"<<det.dy<< std::endl;
        // // std::cout<<"tx1:"<<trk.x <<" ty1:"<<trk.y <<" tx"<<trk.dx<<"ty"<<trk.dy<<std::endl;

        // auto l = std::max(0, int(xx2 - xx1));
        // auto w = std::max(0, int(yy2 - yy1));

        // float det_area = det.dx * det.dy;
        // float trk_area = trk.dx * trk.dy;

        // auto intersection_area = l * w;
        // float union_area = det_area + trk_area - intersection_area;
        // std::cout<<det_area+trk_area<<" "<<intersection_area<<std::endl;
        // auto iou = intersection_area / union_area;

        cv::Rect det_box;
        det_box.x=det.x;
        det_box.y=det.y;
        det_box.width=det.dx;
        det_box.height=det.dy;

        cv::Rect trk_box;
        trk_box.x=trk.x;
        trk_box.y=trk.y;
        trk_box.width=trk.dx;
        trk_box.height=trk.dy;

        cv::Rect intersection_area=det_box&trk_box;
        cv::Rect union_area=det_box|trk_box;
        // std::cout<<"intersection_area:"<<intersection_area.area()<<std::endl;
        // std::cout<<"union_area:"<<union_area.area()<<std::endl;

        if(union_area.area()==0){return 0;}
        float iou = intersection_area.area() *1.0/ union_area.area();
        // std::cout<<"iou:"<<iou<<std::endl;
        return iou;
    }

    float Tracking::CalculateRotateIOU(const BBox& det, const Tracker& track) {
        auto trk = track.GetStateAsBbox();
        
        cv::Point2f det_center(det.x,det.y);
        cv::Size2f det_size(det.dx,det.dy);
        float tmp_det_angle=det.yaw;
        if (tmp_det_angle < 0)
            {tmp_det_angle+=2. * M_PI;}
        float det_angle=tmp_det_angle/(2*M_PI)*360;
        cv::RotatedRect det_rectangle(det_center,det_size,det_angle);

        cv::Point2f trk_center(trk.x,trk.y);
        cv::Size2f trk_size(trk.dx,trk.dy);
        float tmp_trk_angle=trk.yaw;
        if(tmp_trk_angle<0){
            tmp_trk_angle+=2. * M_PI;
        }
        float trk_angle=tmp_trk_angle/(2*M_PI)*360;
        cv::RotatedRect trk_rectangle(trk_center,trk_size,trk_angle);

        float det_area = det_rectangle.size.width * det_rectangle.size.height;
        float trk_area = trk_rectangle.size.width * trk_rectangle.size.height;
        std::vector<cv::Point2f> vertices;
    
        int intersectionType = cv::rotatedRectangleIntersection(det_rectangle, trk_rectangle, vertices);
        if (vertices.size()==0)
            return 0.0;
        else{
            std::vector<cv::Point2f> order_pts;
            // 找到交集（交集的区域），对轮廓的各个点进行排序
            cv::convexHull(cv::Mat(vertices), order_pts, true);
            double inter_area = cv::contourArea(order_pts);
            float iou = (float) (inter_area / (det_area + trk_area - inter_area + 0.0001));
            return iou;
        }

    }

    float Tracking::CalculateLocationDistance(const BBox& det, const Tracker& track){
        auto trk = track.GetStateAsBbox();

        float distance=sqrt((trk.x-det.x)*(trk.x-det.x)+(trk.y-det.y)*(trk.y-det.y));
        
        float distance_score=exp(-distance);
        //std::cout<<"distance_socre:"<<distance_score<<std::endl;

        return distance_score;
    }   

    void Tracking::HungarianMatching(const std::vector<std::vector<float>>& iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association) {
        Matrix<float> matrix(nrows, ncols);
        // Initialize matrix with IOU values
        for (size_t i = 0 ; i < nrows ; i++) {
            for (size_t j = 0 ; j < ncols ; j++) {
                // Multiply by -1 to find max cost
                if (iou_matrix[i][j] != 0) {
                    matrix(i, j) = -iou_matrix[i][j];
                }
                else {
                    matrix(i, j) = 1.0f;
                }
            }
        }

        // Apply Kuhn-Munkres algorithm to matrix.
        Munkres<float> m;
        m.solve(matrix);

        for (size_t i = 0 ; i < nrows ; i++) {
            for (size_t j = 0 ; j < ncols ; j++) {
                association[i][j] = matrix(i, j);
            }
        }

    }

    void Tracking::AssociateDetectionsToTrackers(const std::vector<BBox> &bboxes,
                                                 std::map<int, Tracker>& tracks,
                                                 std::map<int, BBox>& matched,
                                                 std::vector<BBox>& unmatched_det,
                                                 float iou_threshold) {
        // Set all detection as unmatched if no tracks existing
        if (tracks.empty()) {
            for (const auto& det : bboxes) {
                unmatched_det.push_back(det);
            }
            return;
        }

        std::vector<std::vector<float>> iou_matrix;
        iou_matrix.resize(bboxes.size(), std::vector<float>(tracks.size()));

        std::vector<std::vector<float>> association;
        association.resize(bboxes.size(), std::vector<float>(tracks.size()));

        // row - detection, column - tracks
        for (size_t i = 0; i < bboxes.size(); i++) {
            size_t j = 0;
            for (const auto& trk : tracks) {
                float iou_socre=CalculateIou(bboxes[i], trk.second)*0.8;
                float distance_score=CalculateLocationDistance(bboxes[i],trk.second)*0.2;
                iou_matrix[i][j] = iou_socre+distance_score;
                //std::cout<<"iou_matrix:"<<iou_matrix[i][j]<<std::endl;
                //iou_matrix[i][j] = CalculateRotateIOU(bboxes[i], trk.second);
                j++;
            }
        }
        
        // Find association
        HungarianMatching(iou_matrix, bboxes.size(), tracks.size(), association);

        for (size_t i = 0; i < bboxes.size(); i++) {
            bool matched_flag = false;
            size_t j = 0;
            for (const auto& trk : tracks) {
                if (0 == association[i][j]) {
                    // Filter out matched with low IOU
                    if (iou_matrix[i][j] >= filter_threshold_) {//IOU threshold
                        matched[trk.first] = bboxes[i];
                        matched_flag = true;
                    }
                    // It builds 1 to 1 association, so we can break from here
                    break;
                }
                j++;
            }
            // if detection cannot match with any tracks
            if (!matched_flag) {
                unmatched_det.push_back(bboxes[i]);
            }
        }
    }

    int Tracking::Track(std::map<int, Tracker> &tracks, std::vector<BBox> bboxes, int frame_index, int &current_id,
                        lidar_perception::ObjectInfoArray& object_array_msg) {
        for (auto &track : tracks){
            track.second.Predict();
        }
        
        if(bboxes.size()!=0){
            std::map<int, BBox> matched;
            std::vector<BBox> unmatched_det;
            AssociateDetectionsToTrackers(bboxes, tracks, matched, unmatched_det);

            // Update tracks with associated bbox
            for (const auto &match : matched) {
                const auto &ID = match.first;
                tracks[ID].Update(match.second);
            }

            // Create new tracks for unmatched detections
            for (const auto &det : unmatched_det) {
                Tracker tracker;
                tracker.Init(det);
                // Create new track and generate new ID
                tracks[current_id++] = tracker;
            }

            // Delete lose tracked tracks
            for (auto it = tracks.begin(); it != tracks.end();) {
                // if (it->second.GetCoastCycles() > max_coast_cycles_) {
                if (it->second.GetCoastCycles() > max_coast_cycles_) {
                    it = tracks.erase(it);
                } else {
                    it++;
                }
            }
        }

        float velocity_x=0;//gnssInput.vehicleNorthSpeed*cos(gnssInput.heading/360*2*M_PI)+gnssInput.vehicleEarthSpeed*sin(gnssInput.heading/360*2*M_PI);
        float velocity_y=0;//gnssInput.vehicleNorthSpeed*sin(gnssInput.heading/360*2*M_PI)-gnssInput.vehicleEarthSpeed*cos(gnssInput.heading/360*2*M_PI);
        
        // std::cout<<"vehicleNorthSpeed:"<<gnssInput.vehicleNorthSpeed<<std::endl;
        // std::cout<<"vehicleNorthSpeed:"<<gnssInput.vehicleEarthSpeed<<std::endl;
        lidar_perception::TrackingObjectInfo object_info_msg;
        int object_num = 0;
        for (auto &trk : tracks) {
            const auto &bbox = trk.second.GetStateAsBbox();
            const auto &velocity=trk.second.GetStateAsVelocity();
            bool state=trk.second.GetCoastCycles() < max_coast_cycles_ && (trk.second.GetHitStreak() >= min_hits_||frame_index < min_hits_);
            if (trk.second.GetCoastCycles() < max_coast_cycles_ && (trk.second.GetHitStreak() >= min_hits_||frame_index < min_hits_)){
                object_info_msg.id = (uint16_t)trk.first;

                object_info_msg.tk_distance_xv=(int16_t)(bbox.x*128);
                object_info_msg.tk_distance_yv=(int16_t)(bbox.y*128);
                object_info_msg.tk_center_z=(int16_t)(velocity.center_z*128);
                
                if(bbox.dx>=30||bbox.dy>=40){continue;}
                object_info_msg.tk_length = (uint16_t)(bbox.dx*128);
                object_info_msg.tk_width = (uint16_t)(bbox.dy*128);
                object_info_msg.tk_height = (uint16_t)(velocity.height*128);

                object_info_msg.tk_velocity_xv=(int16_t)(velocity.velocity_x+velocity_x)*256;
                object_info_msg.tk_velocity_yv=(int16_t)(velocity.velocity_y+velocity_y)*256;
                object_array_msg.tracking_object_info[object_num] = object_info_msg;
                object_num++;
            }
        }
        object_array_msg.tracking_object_num = object_num;
    }
}