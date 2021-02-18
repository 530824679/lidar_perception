
#include "builder/bbox_fitting_lshape.h"

namespace lidar_perception_ros{

    LShapeBBoxEstimator::LShapeBBoxEstimator(BBoxParam param): bbox_filter_(param)
    {

    }

    LShapeBBoxEstimator::~LShapeBBoxEstimator()
    {

    }

    // tmp add rect
    void LShapeBBoxEstimator::GetMinMax3D(const PointCloudPtr &in_cloud_ptr, PointXYZI<float>& min_point, PointXYZI<float>& max_point, PointXYZI<float>& centroid_point)
    {
        Eigen::Array3f min_pt, max_pt;
        min_pt.setConstant(FLT_MAX);
        max_pt.setConstant(-FLT_MAX);

        PointXYZI<float> centroid(0.0, 0.0, 0.0, 1.0);

        for (size_t i = 0; i < (*in_cloud_ptr).size(); i++)
        {
            Eigen::Array3f pt;
            pt << (*in_cloud_ptr).at(i).GetX(), (*in_cloud_ptr).at(i).GetY(), (*in_cloud_ptr).at(i).GetZ();
            min_pt = min_pt.min (pt);
            max_pt = max_pt.max (pt);

            centroid.SetX(centroid.GetX() + (*in_cloud_ptr)[i].GetX());
            centroid.SetY(centroid.GetY() + (*in_cloud_ptr)[i].GetY());
            centroid.SetZ(centroid.GetZ() + (*in_cloud_ptr)[i].GetZ());
        }

        centroid_point.SetX(centroid.GetX() / (*in_cloud_ptr).size());
        centroid_point.SetY(centroid.GetY() / (*in_cloud_ptr).size());
        centroid_point.SetZ(centroid.GetZ() / (*in_cloud_ptr).size());

        min_point.SetX(min_pt[0]); min_point.SetY(min_pt[1]); min_point.SetZ(min_pt[2]);
        max_point.SetX(max_pt[0]); max_point.SetY(max_pt[1]); max_point.SetZ(max_pt[2]);
    }

    void LShapeBBoxEstimator::Estimate(std::vector<PointCloud> &clusters, std::vector<BBox> &bboxes)
    {
        int i = 0;
        for (int i = 0; i < clusters.size(); i++) {
            //if(i > 0) break;
            auto cluster = clusters[i];
            BBox box{};

            PointCloudPtr cluster_ptr = PointCloudPtr(new PointCloud (cluster));
            if (SearchBasedFitting(cluster_ptr, box)){
                bboxes.push_back(box);
            }else{
                printf("Search Based Fitting false.\n");
            }
        }

        bbox_filter_.ConditionFilter(bboxes);
    }

    bool LShapeBBoxEstimator::SearchBasedFitting(PointCloudPtr &in_cloud_ptr, BBox &box)
    {
        /* -----------tmp add------------*/
        PointXYZI<float> min_point, max_point, centroid_point;
        GetMinMax3D(in_cloud_ptr, min_point, max_point, centroid_point);

        box.rect_x = (min_point.GetX() + max_point.GetX()) / 2.0;
        box.rect_y = (min_point.GetY() + max_point.GetY()) / 2.0;
        box.rect_z = (min_point.GetZ() + max_point.GetZ()) / 2.0;

        constexpr float ep = 0.001;
        box.rect_dx = std::max(max_point.GetX() - min_point.GetX(), ep);
        box.rect_dy = std::max(max_point.GetY() - min_point.GetY(), ep);
        box.rect_dz = std::max(max_point.GetZ() - min_point.GetZ(), ep);

        box.centroid_x = centroid_point.GetX();
        box.centroid_y = centroid_point.GetY();
        box.centroid_z = centroid_point.GetZ();
        /* --------------------------*/

        box.points_num = (*in_cloud_ptr).size();

        float min_z = (*in_cloud_ptr).front().GetZ();
        float max_z = (*in_cloud_ptr).front().GetZ();
        for (auto &p: *in_cloud_ptr){
            if (p.GetZ() < min_z){
                min_z = p.GetZ();
            }
            if (p.GetZ() > max_z){
                max_z = p.GetZ();
            }
        }

        std::vector<std::pair<float, float>> Q;
        const float max_angle = M_PI / 2.0;
        const float angle_resulution = M_PI / 180.0;
        for(float theta = 0; theta < max_angle; theta += angle_resulution){
            Eigen::Vector2d e_1, e_2;
            e_1 << std::cos(theta), std::sin(theta);
            e_2 << -std::sin(theta), std::cos(theta);

            std::vector<float> c_1, c_2;
            for (const auto &point: *in_cloud_ptr) {
                c_1.emplace_back(point.GetX() * e_1.x() + point.GetY() * e_1.y());
                c_2.emplace_back(point.GetX() * e_2.x() + point.GetY() * e_2.y());
            }

            float q = CalcCloseness(c_1, c_2);
            Q.emplace_back(theta, q);
        }

        float dz = max_z - min_z;
        bool ret = CalcBBox(in_cloud_ptr, Q, dz ,box);
        if (ret){
            return true;
        } else{
            printf("Calculate bounding box false.\n");
            return false;
        }
    }

    float LShapeBBoxEstimator::CalcCloseness(const std::vector<float> &C_1, const std::vector<float> &C_2)
    {
        const float min_c_1 = *std::min_element(C_1.begin(), C_1.end());
        const float max_c_1 = *std::max_element(C_1.begin(), C_1.end());
        const float min_c_2 = *std::min_element(C_2.begin(), C_2.end());
        const float max_c_2 = *std::max_element(C_2.begin(), C_2.end());

        std::vector<float> D_1;
        for (const auto& c_1_element : C_1)
        {
            const float v = std::min(max_c_1 - c_1_element, c_1_element - min_c_1);
            D_1.push_back(std::fabs(v));
        }

        std::vector<float> D_2;
        for (const auto& c_2_element : C_2)
        {
            const float v = std::min(max_c_2 - c_2_element, c_2_element - min_c_2);
            D_2.push_back(v * v);
        }

        const float d_min = 0.05;
        const float d_max = 0.50;
        float beta = 0;
        for (size_t i = 0; i < D_1.size(); ++i)
        {
            const float d = std::min(std::max(std::min(D_1.at(i), D_2.at(i)), d_min), d_max);
            //const float d = std::max(std::min(D_1.at(i), D_2.at(i)), d_min);
            beta += 1.0 / d;
        }
        return beta;
    }

    bool LShapeBBoxEstimator::CalcBBox(PointCloudPtr &in_cloud_ptr, std::vector<std::pair<float, float>> &Q, float dz, BBox &box)
    {
        float theta_star;
        float max_q;
        for (size_t i = 0; i < Q.size(); ++i) {
            if (max_q < Q.at(i).second || i == 0) {
                max_q = Q.at(i).second;
                theta_star = Q.at(i).first;
            }
        }

        Eigen::Vector2d e_1_star;
        Eigen::Vector2d e_2_star;
        e_1_star << std::cos(theta_star), std::sin(theta_star);
        e_2_star << -std::sin(theta_star), std::cos(theta_star);

        std::vector<float> C_1_star;
        std::vector<float> C_2_star;
        for (const auto& point: *in_cloud_ptr) {
            C_1_star.emplace_back(point.GetX() * e_1_star.x() + point.GetY() * e_1_star.y());
            C_2_star.emplace_back(point.GetX() * e_2_star.x() + point.GetY() * e_2_star.y());
        }

        const float min_C_1_star = *std::min_element(C_1_star.begin(), C_1_star.end());
        const float max_C_1_star = *std::max_element(C_1_star.begin(), C_1_star.end());
        const float min_C_2_star = *std::min_element(C_2_star.begin(), C_2_star.end());
        const float max_C_2_star = *std::max_element(C_2_star.begin(), C_2_star.end());

        a_.clear();
        b_.clear();
        c_.clear();

        a_.push_back(std::cos(theta_star));
        b_.push_back(std::sin(theta_star));
        c_.push_back(min_C_1_star);

        a_.push_back(-1.0 * std::sin(theta_star));
        b_.push_back(std::cos(theta_star));
        c_.push_back(min_C_2_star);

        a_.push_back(std::cos(theta_star));
        b_.push_back(std::sin(theta_star));
        c_.push_back(max_C_1_star);

        a_.push_back(-1.0 * std::sin(theta_star));
        b_.push_back(std::cos(theta_star));
        c_.push_back(max_C_2_star);

        // calc center of bounding box
        float intersection_x_1 = (b_[0] * c_[1] - b_[1] * c_[0]) / (a_[1] * b_[0] - a_[0] * b_[1]);
        float intersection_y_1 = (a_[0] * c_[1] - a_[1] * c_[0]) / (a_[0] * b_[1] - a_[1] * b_[0]);
        float intersection_x_2 = (b_[2] * c_[3] - b_[3] * c_[2]) / (a_[3] * b_[2] - a_[2] * b_[3]);
        float intersection_y_2 = (a_[2] * c_[3] - a_[3] * c_[2]) / (a_[2] * b_[3] - a_[3] * b_[2]);

        // calc dimention of bounding box
        Eigen::Vector2d e_x;
        Eigen::Vector2d e_y;
        e_x << a_[0] / (std::sqrt(a_[0] * a_[0] + b_[0] * b_[0])), b_[0] / (std::sqrt(a_[0] * a_[0] + b_[0] * b_[0]));
        e_y << a_[1] / (std::sqrt(a_[1] * a_[1] + b_[1] * b_[1])), b_[1] / (std::sqrt(a_[1] * a_[1] + b_[1] * b_[1]));
        Eigen::Vector2d diagonal_vec;
        diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

        CalcRectPoints(box);

        box.rotate = theta_star;
        //box.yaw = std::atan2(e_1_star.y(), e_1_star.x());
        box.x = (intersection_x_1 + intersection_x_2) / 2.0;
        box.y = (intersection_y_1 + intersection_y_2) / 2.0;
        box.z = box.centroid_z;

        constexpr float ep = 0.001;
        box.dx = std::fabs(e_x.dot(diagonal_vec));
        box.dy = std::fabs(e_y.dot(diagonal_vec));
        box.dz = std::max(dz, ep);

        if (box.dx < ep && box.dy < ep)
            return false;
        box.dx = std::max(box.dx, ep);
        box.dy = std::max(box.dy, ep);
        return true;
    }

    void LShapeBBoxEstimator::CalcCrossPoint(const float a0, const float a1, const float b0, const float b1, const float c0, const float c1, float& x, float& y)
    {
        x = (b0 * (-c1) - b1 * (-c0)) / (a0 * b1 - a1 * b0);
        y = (a1 * (-c0) - a0 * (-c1)) / (a0 * b1 - a1 * b0);
    }

    void LShapeBBoxEstimator::CalcRectPoints(BBox &box)
    {
        box.vertex_pts.clear();

        float top_left_x = 0.0, top_left_y = 0.0;
        CalcCrossPoint(a_[0], a_[1], b_[0], b_[1], c_[0], c_[1], top_left_x, top_left_y);
        Point2D pt1(top_left_x, top_left_y);
        box.vertex_pts.push_back(pt1);

        float top_right_x = 0.0, top_right_y = 0.0;
        CalcCrossPoint(a_[1], a_[2], b_[1], b_[2], c_[1], c_[2], top_right_x, top_right_y);
        Point2D pt2(top_right_x, top_right_y);
        box.vertex_pts.push_back(pt2);

        float bottom_left_x = 0.0, bottom_left_y = 0.0;
        CalcCrossPoint(a_[2], a_[3], b_[2], b_[3], c_[2], c_[3], bottom_left_x, bottom_left_y);
        Point2D pt3(bottom_left_x, bottom_left_y);
        box.vertex_pts.push_back(pt3);

        float bottom_right_x = 0.0, bottom_right_y = 0.0;
        CalcCrossPoint(a_[3], a_[0], b_[3], b_[0], c_[3], c_[0], bottom_right_x, bottom_right_y);
        Point2D pt4(bottom_right_x, bottom_right_y);
        box.vertex_pts.push_back(pt4);

        // 矩形框下边沿中心点
        float min_x = 100.0;
        float sum_y = 0.0;
        for(int i = 0; i < 4; i++)
        {
            if(box.vertex_pts[i].x < min_x)
            {
                min_x = box.vertex_pts[i].x;
            }
            sum_y += box.vertex_pts[i].y;
        }
        Point2D pt5(min_x, sum_y/4);
        box.vertex_pts.push_back(pt5);
    }
}