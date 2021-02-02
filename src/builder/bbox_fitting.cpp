
#include "builder/bbox_fitting.h"

namespace lidar_perception_ros{

    BBoxEstimator::BBoxEstimator(BBoxParam param): bbox_filter_(param)
    {
        min_distance_ = param.min_distance_;
        max_distance_ = param.max_distance_;
    }

    BBoxEstimator::~BBoxEstimator()
    {

    }

    void BBoxEstimator::Estimate(std::vector<PointCloud> &clusters, std::vector<BBox> &bboxes, lidar_perception::ObjectInfoArray& object_info_msg)
    {
        int i = 0;
        for (int i = 0; i < clusters.size(); i++) {
            //if(i > 0) break;
            auto cluster = clusters[i];
            BBox box{};

            PointCloudPtr cluster_ptr = PointCloudPtr(new PointCloud (cluster));
            if (HullFitting(cluster_ptr, box)){
                bboxes.push_back(box);
            }else{
                printf("Search Based Fitting false.\n");
            }
        }

        bbox_filter_.ConditionFilter(bboxes);


        lidar_perception::DetectionObjectInfo detection_info_msg;
        int object_num = 0;
        for (auto &box : bboxes) {
            detection_info_msg.dt_center_x=(int16_t)(box.x*128);
            detection_info_msg.dt_center_y=(int16_t)(box.y*128);
            detection_info_msg.dt_center_z=(int16_t)(box.z*128);

            std::cout << "x: " << box.x*128 << "y: " << box.y*128 << std::endl;

            detection_info_msg.dt_length = (uint16_t)(box.dx*128);
            detection_info_msg.dt_width = (uint16_t)(box.dy*128);
            detection_info_msg.dt_height = (uint16_t)(box.dz*128);

            detection_info_msg.dt_yaw = (uint16_t)(box.yaw*128);

//            detection_info_msg.dt_distance_xv=(int16_t)(bbox.x*128);
//            detection_info_msg.dt_distance_yv=(int16_t)(bbox.y*128);

             object_info_msg.detection_object_info[object_num] = detection_info_msg;
             object_num++;
            }
        object_info_msg.detection_object_num = object_num;
    }

    bool BBoxEstimator::SearchBasedFitting(PointCloudPtr &in_cloud_ptr, BBox &box)
    {
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

    float BBoxEstimator::CalcCloseness(const std::vector<float> &C_1, const std::vector<float> &C_2)
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
            D_2.push_back(std::fabs(v));
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

    bool BBoxEstimator::CalcBBox(const PointCloudPtr &in_cloud_ptr, std::vector<std::pair<float, float>> &Q, float dz, BBox &box)
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

        const float a_1 = std::cos(theta_star);
        const float b_1 = std::sin(theta_star);
        const float c_1 = min_C_1_star;
        const float a_2 = -1.0 * std::sin(theta_star);
        const float b_2 = std::cos(theta_star);
        const float c_2 = min_C_2_star;
        const float a_3 = std::cos(theta_star);
        const float b_3 = std::sin(theta_star);
        const float c_3 = max_C_1_star;
        const float a_4 = -1.0 * std::sin(theta_star);
        const float b_4 = std::cos(theta_star);
        const float c_4 = max_C_2_star;

        // calc center of bounding box
        float intersection_x_1 = (b_1 * c_2 - b_2 * c_1) / (a_2 * b_1 - a_1 * b_2);
        float intersection_y_1 = (a_1 * c_2 - a_2 * c_1) / (a_1 * b_2 - a_2 * b_1);
        float intersection_x_2 = (b_3 * c_4 - b_4 * c_3) / (a_4 * b_3 - a_3 * b_4);
        float intersection_y_2 = (a_3 * c_4 - a_4 * c_3) / (a_3 * b_4 - a_4 * b_3);

        // calc dimention of bounding box
        Eigen::Vector2d e_x;
        Eigen::Vector2d e_y;
        e_x << a_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1)), b_1 / (std::sqrt(a_1 * a_1 + b_1 * b_1));
        e_y << a_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2)), b_2 / (std::sqrt(a_2 * a_2 + b_2 * b_2));
        Eigen::Vector2d diagonal_vec;
        diagonal_vec << intersection_x_1 - intersection_x_2, intersection_y_1 - intersection_y_2;

        box.yaw =   M_PI-theta_star;
        //box.yaw = std::atan2(e_1_star.y(), e_1_star.x());
        box.x = (intersection_x_1 + intersection_x_2) / 2.0;
        box.y = (intersection_y_1 + intersection_y_2) / 2.0;
        box.z = CalcCloudCentroid(in_cloud_ptr).z();

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

    Eigen::Array3f BBoxEstimator::CalcCloudCentroid(const PointCloudPtr &in_cloud_ptr)
    {
        Eigen::Array3f min_pt, max_pt;
        min_pt.setConstant(FLT_MAX);
        max_pt.setConstant(-FLT_MAX);

        //min_pt << (*in_cloud_ptr).at(0).GetX(), (*in_cloud_ptr).at(0).GetY(), (*in_cloud_ptr).at(0).GetZ();
        //max_pt << (*in_cloud_ptr).at(0).GetX(), (*in_cloud_ptr).at(0).GetY(), (*in_cloud_ptr).at(0).GetZ();

        for (size_t i = 1; i < (*in_cloud_ptr).size(); i++) {
            Eigen::Array3f pt;
            pt << (*in_cloud_ptr).at(i).GetX(), (*in_cloud_ptr).at(i).GetY(), (*in_cloud_ptr).at(i).GetZ();
            min_pt = pt.min(min_pt);
            max_pt = pt.max(max_pt);
        }

        Eigen::Array3f centroid = (min_pt + max_pt) / 2.0f;
        return centroid;
    }

    bool BBoxEstimator::AABBFitting(PointCloudPtr &in_cloud_ptr, BBox &box)
    {
        PointXYZI<float> min_point, max_point;
        GetMinMax3D(in_cloud_ptr, min_point, max_point);

        box.x = (min_point.GetX() + max_point.GetX()) / 2.0;
        box.y = (min_point.GetY() + max_point.GetY()) / 2.0;
        box.z = (min_point.GetZ() + max_point.GetZ()) / 2.0;

        constexpr float ep = 0.001;
        box.dx = std::max(max_point.GetX() - min_point.GetX(), ep);
        box.dy = std::max(max_point.GetY() - min_point.GetY(), ep);
        box.dz = std::max(max_point.GetZ() - min_point.GetZ(), ep);

        return true;
    }

    bool BBoxEstimator::HullFitting(PointCloudPtr &in_cloud_ptr, BBox &box)
    {
        PointCloud hull;
        convex_hull_.GrahamScan(in_cloud_ptr, hull);
        convex_hull_.MinAreaRect(hull, box);

        float min_z = FLT_MAX;
        float max_z = -FLT_MAX;

        for (int i = 0; i < (*in_cloud_ptr).size(); i++)
        {
            if((*in_cloud_ptr)[i].GetZ() < min_z)
            {
                min_z = (*in_cloud_ptr)[i].GetZ();
            }

            if((*in_cloud_ptr)[i].GetZ() > max_z)
            {
                max_z = (*in_cloud_ptr)[i].GetZ();
            }
        }

        box.z = (max_z + min_z) / 2.0;

        constexpr float ep = 0.001;
        box.dz = std::max(max_z -min_z, ep);
    }

    void BBoxEstimator::GetMinMax3D(const PointCloudPtr &in_cloud_ptr, PointXYZI<float>& min_point, PointXYZI<float>& max_point)
    {
        Eigen::Array3f min_pt, max_pt;
        min_pt.setConstant(FLT_MAX);
        max_pt.setConstant(-FLT_MAX);

        for (size_t i = 0; i < (*in_cloud_ptr).size(); i++)
        {
            Eigen::Array3f pt;
            pt << (*in_cloud_ptr).at(i).GetX(), (*in_cloud_ptr).at(i).GetY(), (*in_cloud_ptr).at(i).GetZ();
            min_pt = min_pt.min (pt);
            max_pt = max_pt.max (pt);
        }

        min_point.SetX(min_pt[0]); min_point.SetY(min_pt[1]); min_point.SetZ(min_pt[2]);
        max_point.SetX(max_pt[0]); max_point.SetY(max_pt[1]); max_point.SetZ(max_pt[2]);
    }

}