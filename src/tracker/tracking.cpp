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
        kernel_size_ = param.kernel_size_;
        iou_weight_ = param.iou_weight_;
        rotate_iou_weight_ = param.rotate_iou_weight_;
        distance_weight_ = param.distance_weight_;
        centroid_weight_ = param.centroid_weight_;
        pointnum_weight_ = param.pointnum_weight_;
        use_mutil_threshold_ = param.use_mutil_threshold_;
        first_threshold_ = param.first_threshold_;
        second_threshold_ = param.second_threshold_;
        third_threshold_ = param.third_threshold_;
        tracking_distance_[4] = param.tracking_distance_[4];
        
    }

    Tracking::~Tracking() {

    }

    void Tracking::Process(std::vector<BBox> bboxes)
    {
        Track(tracks_, bboxes, frame_index_, current_id_);
        frame_index_++;
    }

    Rect intersection_area_cal (Rect a, Rect b){
        Rect c(a.x,a.y,a.width,a.height);
        float x1 = std::max(c.x, b.x);
        float y1 = std::max(c.y, b.y);
        c.width = std::min(c.x + c.width, b.x + b.width) - x1;
        c.height = std::min(c.y + c.height, b.y + b.height) - y1;
        c.x = x1;
        c.y = y1;
        if( c.width <= 0 || c.height <= 0 )
            c = Rect();
        return c;
    }

    Rect union_area_cal (Rect a, Rect b){
        Rect c(a.x,a.y,a.width,a.height);
        if (c.width <= 0|| c.height <= 0 ) {
            c = b;
        }
        else if (!(b.width <= 0 || b.height <= 0)) {
            float x1 = std::min(c.x, b.x);
            float y1 = std::min(c.y, b.y);
            c.width = std::max(c.x + c.width, b.x + b.width) - x1;
            c.height = std::max(c.y + c.height, b.y + b.height) - y1;
            c.x = x1;
            c.y = y1;
        }
        return c;
        }

    float Tracking::CalculateIou(const BBox& det, const Tracker& track) {
        auto trk = track.GetStateAsBbox();

        Rect det_box(det.rect_x,det.rect_y,det.rect_dx,det.rect_dy);
        Rect trk_box(trk.center_x,trk.center_y,trk.width,trk.length);

        Rect intersection_area = intersection_area_cal(det_box,trk_box);

        Rect union_area= union_area_cal(det_box,trk_box);
        float iou = static_cast<float> (intersection_area.area()/static_cast<float>(union_area.area()+ 0.0001));
        return iou;
    }

    float Tracking::CalculateRotateIOU(const BBox& det, const Tracker& track) {
        auto trk = track.GetStateAsInputBbox();
        
        Point2D det_center(det.x,det.y);
        Size2D det_size(det.dx,det.dy);
        float tmp_det_angle=det.yaw;
        if (tmp_det_angle < 0)
            {tmp_det_angle+=2. * M_PI;}
        float det_angle=tmp_det_angle/(2*M_PI)*360;
        RotatedRect det_rectangle(det_center,det_size,det_angle);

        Point2D trk_center(trk.center_x,trk.center_y);
        Size2D trk_size(trk.width,trk.length);
        float tmp_trk_angle=trk.angle;
        if(tmp_trk_angle<0){
            tmp_trk_angle+=2. * M_PI;
        }
        float trk_angle=tmp_trk_angle/(2*M_PI)*360;
        RotatedRect trk_rectangle(trk_center,trk_size,trk_angle);

        float det_area = det_rectangle.size.width * det_rectangle.size.height;
        float trk_area = trk_rectangle.size.width * trk_rectangle.size.height;
        std::vector<Point2D> vertices;
    
        int intersectionType = rotated_iou_.rotatedRectangleIntersection(det_rectangle, trk_rectangle, vertices);

        if (vertices.size()==0)
            return 0.0;
        else{
            std::vector<Point2D> order_pts;
            // 找到交集（交集的区域），对轮廓的各个点进行排序
            rotated_iou_.convexHull(vertices,order_pts);
            double inter_area = rotated_iou_.counterArea(order_pts);
            float iou = (float) (inter_area) / (det_area + trk_area - inter_area + 0.0001);
            return iou;
        }

    }

    float Tracking::CalculateLocationDistance(const BBox& det, const Tracker& track){
        auto trk = track.GetStateAsBbox();
        float det_bottom_center_x = det.rect_x - det.rect_dx/2;
        float det_bottom_center_y = det.rect_y;
        float trk_bottom_center_x = trk.center_x - trk.length/2;
        float trk_bottom_center_y = trk.center_y;

        // float det_bottom_center_x = det.rect_x;
        // float det_bottom_center_y = det.rect_y;
        // float trk_bottom_center_x = trk.center_x;
        // float trk_bottom_center_y = trk.center_y;

        float bottom_center_distance = sqrt((trk_bottom_center_x-det_bottom_center_x)*(trk_bottom_center_x-det_bottom_center_x)+
                              (trk_bottom_center_y-det_bottom_center_y)*(trk_bottom_center_y-det_bottom_center_y));

        //float distance = sqrt((trk.center_x-det.rect_x)*(trk.center_x-det.rect_x)+(trk.center_y-det.rect_y)*(trk.center_y-det.rect_y));
        
        //float distance_score = 1/(1+exp(-distance));
        float bottom_center_distance_score = exp(-bottom_center_distance);
        return bottom_center_distance_score;
    }

    float Tracking::CalculateCentroidDistance(const BBox& det, const Tracker& track){
        auto trk = track.GetStateAsInputBbox();
        float det_centroid_x = det.centroid_x;
        float det_centroid_y = det.centroid_y;
        float trk_centroid_x = trk.centroid_x;
        float trk_centroid_y = trk.centroid_y;
        float centroid_distance = sqrt((trk_centroid_x-det_centroid_x)*(trk_centroid_x-det_centroid_x)+
                              (trk_centroid_y-det_centroid_y)*(trk_centroid_y-det_centroid_y));
        
        float centroid_distance_score = exp(-centroid_distance);
        return centroid_distance_score;
        
    }

    float Tracking::CalculatePointnumDistance(const BBox& det, const Tracker& track){
        auto trk = track.GetStateAsInputBbox();
        float trk_point_num = trk.points_num;
        float det_point_num = det.points_num;

        float pointnum_distance = abs(det.points_num-trk.points_num);
        float pointnum_distance_score = 1-pointnum_distance / std::max(trk_point_num,det_point_num);
        return pointnum_distance_score;
        
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
                float iou_score = CalculateIou(bboxes[i], trk.second)*iou_weight_;
                float rotate_iou_score = CalculateRotateIOU(bboxes[i], trk.second)*rotate_iou_weight_;
                float distance_score = CalculateLocationDistance(bboxes[i], trk.second)*distance_weight_;
                float centroid_score = CalculateCentroidDistance(bboxes[i],trk.second)*centroid_weight_;
                float pointnums_score = CalculatePointnumDistance(bboxes[i],trk.second)*pointnum_weight_;
                iou_matrix[i][j] = iou_score+distance_score+rotate_iou_score+centroid_score+pointnums_score;
                
                // float iou_score=CalculateIou(bboxes[i], trk.second)*0.1;
                // float rotate_iou_score = CalculateRotateIOU(bboxes[i], trk.second)*0.1;
                // float distance_score = CalculateLocationDistance(bboxes[i],trk.second)*0.5;
                // float centroid_score = 0.3;
                // iou_matrix[i][j] = iou_score+distance_score+rotate_iou_score+centroid_score;
                // std::cout<<"iou_matrix:"<<iou_matrix[i][j]<<std::endl;
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
                    if(use_mutil_threshold_){
                    if(tracking_distance_[2]<bboxes[i].x&&bboxes[i].x<tracking_distance_[3]){
                        filter_threshold_ = third_threshold_;
                    }else if(tracking_distance_[1]<bboxes[i].x&&bboxes[i].x<=tracking_distance_[2]){
                        filter_threshold_ = second_threshold_;
                    }else if(tracking_distance_[0]<bboxes[i].x&&bboxes[i].x<=tracking_distance_[1]){
                        filter_threshold_ = first_threshold_;
                    }
                    }
                    
                    // Filter out matched with low IOU
                    if (iou_matrix[i][j] >= filter_threshold_) {//IOU threshold
                    //std::cout<<"filter_threshold_:"<<filter_threshold_<<std::endl;
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

    int Tracking::Track(std::map<int, Tracker> &tracks, std::vector<BBox> bboxes, int frame_index, int &current_id) {
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
    }
}