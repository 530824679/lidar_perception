#include "detect/curb_detect.h"

namespace lidar_perception_ros
{
    CurbDetect::CurbDetect(ROIParam roi_param, CurbParam curb_param)
    {
        min_x_ = roi_param.roi_x_min_;
        max_x_ = roi_param.roi_x_max_;
        min_y_ = roi_param.roi_y_min_;
        max_y_ = roi_param.roi_y_max_;
        min_z_ = roi_param.roi_z_min_;
        max_z_ = roi_param.roi_z_max_;

        min_grid_z_ = curb_param.min_grid_z_;
        max_grid_z_ = curb_param.max_grid_z_;
        min_ground_plane_z_ = curb_param.min_ground_plane_z_;
        max_ground_plane_z_ = curb_param.max_ground_plane_z_;
        max_ground_plane_x_ = curb_param.max_ground_plane_x_;
        min_ground_plane_i_ = curb_param.min_ground_plane_i_;
        max_ground_plane_i_ = curb_param.max_ground_plane_i_;
        max_iterations_ = curb_param.max_iterations_;
        min_ground_plane_points_ = curb_param.min_ground_plane_points_;
        plane_dist_threshold_ = curb_param.plane_dist_threshold_;
        grid_row_ = curb_param.grid_row_;
        grid_col_ = curb_param.grid_column_;
        route_threshold_ = curb_param.route_threshold_;

        height_max_upper_threshold_ = curb_param.height_max_upper_threshold_;
        height_max_lower_threshold_ = curb_param.height_max_lower_threshold_;
        height_diff_upper_threshold_ = curb_param.height_diff_upper_threshold_;
        height_diff_lower_threshold_ = curb_param.height_diff_lower_threshold_;

        min_curb_points_ = curb_param.min_curb_points_;
        min_left_curb_points_ = curb_param.min_left_curb_points_;
        min_right_curb_points_ = curb_param.min_right_curb_points_;
        line_dist_threshold_ = curb_param.line_dist_threshold_;
        angle_threshold_ = curb_param.angle_threshold_;
        points_threshold_left_ = curb_param.points_threshold_left_;
        points_threshold_right_ = curb_param.points_threshold_right_;
        not_tracking_max_ = curb_param.not_tracking_max_;

        is_detected_left_ = false;
        is_detected_right_ = false;
        is_init_tracker_left_ = false;
        is_init_tracker_right_ = false;
        not_detected_num_left_ = 0;
        not_detected_num_right_ = 0;

        is_curb_pts_ = false;
    }

    CurbDetect::~CurbDetect()
    {
    }

    void CurbDetect::Detect(const PointCloudPtr& input_cloud_ptr, PointCloudPtr& out_cloud_ptr, Eigen::Vector4d& plane_coefficients)
    {
        // @name:    CurbDetect
        // @summary: To detect curb
        // @input:   input_cloud_ptr,plane_coefficients
        // @param:   voxel_x_,voxel_y_,voxel_z_
        // @return:  out_cloud_ptr
        PointCloudPtr curb_cloud_ptr(new PointCloud);
        PointCloudPtr curb_cloud_left_ptr(new PointCloud);
        PointCloudPtr curb_cloud_right_ptr(new PointCloud);

        // for line
        Eigen::Vector2d detected_line_left, detected_line_right;
        // for tracking
        Eigen::Vector2d tracked_line_left, tracked_line_right;

        is_get_ground_plane_ = GetGroundPlaneCoeffs(input_cloud_ptr, plane_coefficients);

        if (is_get_ground_plane_)
        {
            // select out roadside points
            is_curb_pts_ = GetCurbPoints(input_cloud_ptr, plane_coefficients, curb_cloud_ptr);

            if (is_curb_pts_)
            {
                // separate points to left and right
                SeparatePointsToFit(curb_cloud_ptr, curb_cloud_left_ptr, curb_cloud_right_ptr);

                // line fitting using ransac
                FitLineRansac(curb_cloud_left_ptr, curb_cloud_right_ptr, detected_line_left, detected_line_right);
            }
            else
            {
                is_detected_left_ = false;
                is_detected_right_ = false;
                not_detected_num_left_++;
                not_detected_num_right_++;
            }

            // line tracking using kalman
            LineTracking(detected_line_left, detected_line_right, tracked_line_left, tracked_line_right);
        }
        else
        {
            is_detected_left_ = false;
            is_detected_right_ = false;
            not_detected_num_left_++;
            not_detected_num_right_++;
        }

        // filter points with roadside
        GetPointsWithRoadside(input_cloud_ptr, out_cloud_ptr, tracked_line_left, tracked_line_right);
    }

    bool CurbDetect::GetGroundPlaneCoeffs(const PointCloudPtr& input_cloud_ptr,
                                          Eigen::Vector4d& plane_coefficients)
    {
        // @name:    GetGroundPlaneCoeffs
        // @summary: To get ground plane coeffs
        // @input:   input_cloud_ptr
        // @param:   NULL
        // @return:  bool, plane_coefficients
        PointCloud pc_filter;
        for (size_t i = 0; i < (*input_cloud_ptr).size(); i++)
        {
            float pt_x = (*input_cloud_ptr)[i].GetX();
            float pt_z = (*input_cloud_ptr)[i].GetZ();
            float pt_i = (*input_cloud_ptr)[i].GetI();

            if (pt_z >= min_ground_plane_z_ && pt_z <= max_ground_plane_z_ &&
                pt_i >= min_ground_plane_i_ && pt_i <= max_ground_plane_i_ &&
                pt_x <= max_ground_plane_x_)
            {
                pc_filter.push_back((*input_cloud_ptr)[i]);
            }
        }
        // cerr << "PointCloud after filtering has " << pc_filter.size() << " data points." << endl;

        if (pc_filter.size() > 0)
        {
            int points_num = pc_filter.size();
            int inliers_max = 0;

            int index_1 = 0;
            int index_2 = 0;
            int index_3 = 0;

            for (int k = 0; k < max_iterations_;)
            {
                index_1 = rand() % points_num;
                index_2 = rand() % points_num;
                index_3 = rand() % points_num;

                Eigen::Array3f pt1, pt2, pt3;
                pt1[0] = pc_filter[index_1].GetX();
                pt1[1] = pc_filter[index_1].GetY();
                pt1[2] = pc_filter[index_1].GetZ();
                pt2[0] = pc_filter[index_2].GetX();
                pt2[1] = pc_filter[index_2].GetY();
                pt2[2] = pc_filter[index_2].GetZ();
                pt3[0] = pc_filter[index_3].GetX();
                pt3[1] = pc_filter[index_3].GetY();
                pt3[2] = pc_filter[index_3].GetZ();

                // Compute the segment values (in 3d) between p2 and p1
                Eigen::Array3f pt2pt1 = pt2 - pt1;
                // Compute the segment values (in 3d) between p3 and p1
                Eigen::Array3f pt3pt1 = pt3 - pt1;

                // Avoid some crashes by checking for collinearity here
                Eigen::Array3f dy1dy2 = pt3pt1 / pt2pt1;
                if ((dy1dy2[0] == dy1dy2[1]) && (dy1dy2[2] == dy1dy2[1]))
                {
                    continue;
                }

                // Compute the plane coefficients from the 3 given points in a straightforward manner
                // calculate the plane normal n = (p2-p1) x (p3-p1) = cross (p2-p1, p3-p1)
                Eigen::Vector4d coeff;
                coeff[0] = pt2pt1[1] * pt3pt1[2] - pt2pt1[2] * pt3pt1[1];
                coeff[1] = pt2pt1[2] * pt3pt1[0] - pt2pt1[0] * pt3pt1[2];
                coeff[2] = pt2pt1[0] * pt3pt1[1] - pt2pt1[1] * pt3pt1[0];
                coeff[3] = 0;

                // Normalize
                coeff.normalize();

                coeff[3] = -1 * (coeff[0] * pt1[0] + coeff[1] * pt1[1] + coeff[2] * pt1[2]);

                int inliers = 0;

                for (int index = 0; index < points_num; index++)
                {
                    float x = pc_filter[index].GetX();
                    float y = pc_filter[index].GetY();
                    float z = pc_filter[index].GetZ();
                    float dist = fabs(coeff[0] * x + coeff[1] * y + coeff[2] * z + coeff[3]);

                    if (dist < plane_dist_threshold_)
                    {
                        inliers++;
                    }
                }

                if (inliers > inliers_max)
                {
                    inliers_max = inliers;
                    plane_coefficients = coeff;
                }

                k++;
            }

            if (inliers_max >= min_ground_plane_points_)
            {
                // std::cerr << "inliers_max: " << inliers_max << std::endl;
                // std::cerr << "plane_coefficients: " << plane_coefficients << std::endl;
                return true;
            }
            else
            {
                // std::cerr << "inliers_max: " << inliers_max << std::endl;
                // printf("Too few points to estimate ground plane.\n");
                return false;
            }
        }
    }

    bool CurbDetect::GetCurbPoints(const PointCloudPtr& input_cloud_ptr, Eigen::Vector4d& plane_coefficients, PointCloudPtr& curb_cloud_ptr)
    {
        PointCloudPtr pc_filter(new PointCloud);
        ROIFilter filter_z;
        filter_z.SetInputCloud(input_cloud_ptr);
        filter_z.SetFilterLimits(min_x_, max_x_, min_y_, max_y_, min_grid_z_, max_grid_z_);
        filter_z.Filter(*pc_filter);
        // printf("PointCloud after z filtering has [%d] data points.\n", (*pc_filter).size());

        Grid grid[grid_row_][grid_col_];

        for (unsigned int count = 0; count < (*pc_filter).size(); count++)
        {
            if (std::isnan((*pc_filter)[count].GetX()))
                continue;

            float x = (*pc_filter)[count].GetX();
            float y = (*pc_filter)[count].GetY();
            float z = (*pc_filter)[count].GetZ();
            float intensity = (*pc_filter)[count].GetI();

            float dis = std::abs(x * plane_coefficients[0] + y * plane_coefficients[1] + z * plane_coefficients[2] + plane_coefficients[3]);

            int i = int((x - min_x_) / ((max_x_ - min_x_) / grid_row_));
            int j = int((y - min_y_) / ((max_y_ - min_y_) / grid_col_));

            grid[i][j].points_num_++;

            if (grid[i][j].points_num_ == 1)
            {
                grid[i][j].max_height_ = dis;
                grid[i][j].min_height_ = dis;
                grid[i][j].intensity_ave_ = intensity;
            }
            else
            {
                if (dis > grid[i][j].max_height_)
                    grid[i][j].max_height_ = dis;

                if (dis < grid[i][j].min_height_)
                    grid[i][j].min_height_ = dis;

                grid[i][j].height_diff_ = grid[i][j].max_height_ - grid[i][j].min_height_;

                // Caculate mean intensity
                float i_ave = grid[i][j].intensity_ave_;
                int n = grid[i][j].points_num_;
                grid[i][j].intensity_ave_ = (i_ave * (n - 1) + intensity) / n;
            }

            grid[i][j].grid_cloud_.push_back((*pc_filter)[count]);
        }

        for (int grid_i = 0; grid_i < grid_row_; grid_i++)
        {
            for (int grid_j = 1; grid_j < grid_col_ - 1; grid_j++)
            {
                if (grid[grid_i][grid_j].points_num_ >= 3)
                {
                    if (grid[grid_i][grid_j].max_height_ >= height_max_lower_threshold_ &&
                        grid[grid_i][grid_j].max_height_ <= height_max_upper_threshold_ &&
                        grid[grid_i][grid_j].height_diff_ >= height_diff_lower_threshold_ &&
                        grid[grid_i][grid_j].height_diff_ <= height_diff_upper_threshold_)
                    {
                        // // SGMW
                        // float i_diff1 = grid[grid_i][grid_j].intensity_ave_ - grid[grid_i][grid_j - 1].intensity_ave_;
                        // float i_diff2 = grid[grid_i][grid_j].intensity_ave_ - grid[grid_i][grid_j + 1].intensity_ave_;
                        // if (abs(i_diff1) > 10 || abs(i_diff2) > 10)
                        // {
                        //     grid[grid_i][grid_j].is_roadside_ = true;
                        // }

                        // iMotion
                        grid[grid_i][grid_j].is_roadside_ = true;
                    }
                }
            }
        }
        for (int grid_i = 0; grid_i < grid_row_; grid_i++)
        {
            for (int grid_j = 1; grid_j < grid_col_ - 1; grid_j++)
            {
                if (grid[grid_i][grid_j].is_roadside_)
                {
                    int curb_grid_num = 0;
                    for (int i = 0; i < grid_row_; i++)
                    {
                        if (grid[i][grid_j].is_roadside_ || grid[i][grid_j - 1].is_roadside_ || grid[i][grid_j + 1].is_roadside_)
                        {
                            curb_grid_num++;
                        }
                    }

                    if (curb_grid_num >= 3)
                    {
                        for (int index = 0; index < (grid[grid_i][grid_j].grid_cloud_).size(); index++)
                        {
                            (*curb_cloud_ptr).push_back(grid[grid_i][grid_j].grid_cloud_[index]);
                        }
                    }
                }
            }
        }
        // printf("[%s]: curb cloud size is [%d] points.\n", __func__, (*curb_cloud_ptr).size());

        if ((*curb_cloud_ptr).size() >= min_curb_points_)
        {
            return true;
        }
        else
        {
            // printf("[%s]: Too few points of roadside.\n", __func__);
            return false;
        }
    }

    void CurbDetect::SeparatePointsToFit(const PointCloudPtr& curb_cloud_ptr, PointCloudPtr& curb_cloud_left_ptr, PointCloudPtr& curb_cloud_right_ptr)
    {
        ROIFilter filter_y;
        filter_y.SetInputCloud(curb_cloud_ptr);
        filter_y.SetFilterLimits(min_x_, max_x_, 0, max_y_, min_z_, max_z_);
        filter_y.Filter(*curb_cloud_left_ptr);
        // printf("[%s]: curb_cloud_left_ptr has [%d] data points.\n", __func__, (*curb_cloud_left_ptr).size());

        filter_y.SetInputCloud(curb_cloud_ptr);
        filter_y.SetFilterLimits(min_x_, max_x_, min_y_, 0, min_z_, max_z_);
        filter_y.Filter(*curb_cloud_right_ptr);
        // printf("[%s]: curb_cloud_right_ptr has [%d] data points.\n", __func__, (*curb_cloud_right_ptr).size());
    }

    void CurbDetect::FitLineRansac(const PointCloudPtr& curb_cloud_left_ptr, const PointCloudPtr& curb_cloud_right_ptr, Eigen::Vector2d& detected_line_left, Eigen::Vector2d& detected_line_right)
    {
        int index_1 = 0;
        int index_2 = 0;
        Eigen::Vector3f x_axis(1, 0, 0);

        if ((*curb_cloud_left_ptr).size() >= min_left_curb_points_)
        {
            int left_num = (*curb_cloud_left_ptr).size();
            int left_inliers_max = 0;
            Eigen::MatrixXd left_line(6, 1);

            for (int k = 0; k < max_iterations_; k++)
            {
                while (index_1 == index_2)
                {
                    index_1 = rand() % left_num;
                    index_2 = rand() % left_num;
                }
                PointXYZI<float> point_1 = (*curb_cloud_left_ptr)[index_1];
                PointXYZI<float> point_2 = (*curb_cloud_left_ptr)[index_2];

                // m,n,p
                Eigen::Vector3f p1p2(point_2.GetX() - point_1.GetX(), point_2.GetY() - point_1.GetY(), point_2.GetZ() - point_1.GetZ());
                p1p2.normalize();
                float angle = acos(p1p2.dot(x_axis));

                if (fabs(angle) <= angle_threshold_ * M_PI / 180.0)
                {
                    int inliers = 0;

                    for (int i = 0; i < left_num; i++)
                    {
                        PointXYZI<float> point_3 = (*curb_cloud_left_ptr)[i];
                        Eigen::Vector3f p1p3(point_3.GetX() - point_1.GetX(), point_3.GetY() - point_1.GetY(), point_3.GetZ() - point_1.GetZ());

                        float dis = (p1p2.cross(p1p3)).norm();
                        if (fabs(dis) <= line_dist_threshold_)
                            inliers += 1;
                    }

                    if (inliers >= left_inliers_max)
                    {
                        left_line << point_1.GetX(), point_1.GetY(), point_1.GetZ(), p1p2[0], p1p2[1], p1p2[2];
                        left_inliers_max = inliers;
                    }
                }
                index_1 = index_2;
            }

            // printf("[%s]: inliers_left size: [%d].\n", __func__, left_inliers_max);
            if (left_inliers_max >= points_threshold_left_)
            {
                detected_line_left[0] = left_line(4, 0) / left_line(3, 0);
                detected_line_left[1] = left_line(1, 0) - detected_line_left[0] * left_line(0, 0);
                // printf("[%s]: left_k: [%f], left_b: [%f]\n", __func__, detected_line_left[0], detected_line_left[1]);

                is_detected_left_ = true;
                not_detected_num_left_ = 0;
            }
            else
            {
                // printf("[%s]: Too few points to estimate road line.\n", __func__);
                is_detected_left_ = false;
                not_detected_num_left_++;
            }
        }
        else
        {
            // printf("[%s]: Too few points to estimate road line.\n", __func__);
            is_detected_left_ = false;
            not_detected_num_left_++;
        }

        if ((*curb_cloud_right_ptr).size() >= min_right_curb_points_)
        {
            int right_num = (*curb_cloud_right_ptr).size();
            int right_inliers_max = 0;
            Eigen::MatrixXd right_line(6, 1);

            for (int k = 0; k < max_iterations_; k++)
            {
                while (index_1 == index_2)
                {
                    index_1 = rand() % right_num;
                    index_2 = rand() % right_num;
                }
                PointXYZI<float> point_1 = (*curb_cloud_right_ptr)[index_1];
                PointXYZI<float> point_2 = (*curb_cloud_right_ptr)[index_2];

                // m,n,p
                Eigen::Vector3f p1p2(point_2.GetX() - point_1.GetX(), point_2.GetY() - point_1.GetY(), point_2.GetZ() - point_1.GetZ());
                p1p2.normalize();
                float angle = acos(p1p2.dot(x_axis));

                if (abs(angle) <= angle_threshold_ * M_PI / 180.0)
                {
                    int inliers = 0;

                    for (int i = 0; i < right_num; i++)
                    {
                        PointXYZI<float> point_3 = (*curb_cloud_right_ptr)[i];
                        Eigen::Vector3f p1p3(point_3.GetX() - point_1.GetX(), point_3.GetY() - point_1.GetY(), point_3.GetZ() - point_1.GetZ());

                        float d = (p1p2.cross(p1p3)).norm();
                        if (fabs(d) <= line_dist_threshold_)
                            inliers += 1;
                    }

                    if (inliers >= right_inliers_max)
                    {
                        right_line << point_1.GetX(), point_1.GetY(), point_1.GetZ(), p1p2[0], p1p2[1], p1p2[2];
                        right_inliers_max = inliers;
                    }
                }
                index_1 = index_2;
            }

            // printf("[%s]: inliers_right size: [%d].\n", __func__, right_inliers_max);
            if (right_inliers_max >= points_threshold_right_)
            {
                detected_line_right[0] = right_line(4, 0) / right_line(3, 0);
                detected_line_right[1] = right_line(1, 0) - detected_line_right[0] * right_line(0, 0);
                // printf("[%s]: right_k: [%f], right_b: [%f]\n", __func__, detected_line_right[0], detected_line_right[1]);

                is_detected_right_ = true;
                not_detected_num_right_ = 0;
            }
            else
            {
                // printf("[%s]: Too few points to estimate road line.\n", __func__);
                is_detected_right_ = false;
                not_detected_num_right_++;
            }
        }
        else
        {
            // printf("[%s]: Too few points to estimate road line.\n", __func__);
            is_detected_right_ = false;
            not_detected_num_right_++;
        }
    }

    void CurbDetect::LineTracking(Eigen::Vector2d& detected_line_left,
                                  Eigen::Vector2d& detected_line_right,
                                  Eigen::Vector2d& tracked_line_left,
                                  Eigen::Vector2d& tracked_line_right)
    {
        // Condition of is_init_tracker_left_ == false :
        // 1, First frame
        // 2, First line detected after turning left or right
        // 3, No line detected for more than 5 frames
        if (!is_init_tracker_left_ && is_detected_left_)
        {
            tracker_left_.Init(detected_line_left);
            is_init_tracker_left_ = true;

            tracked_line_left = detected_line_left;

            // printf("[%s]: tracked_left_k [%f], tracked_left_b: [%f].\n", __func__, tracked_line_left[0], tracked_line_left[1]);
        }
        else
        {
            if (is_detected_left_)
            {
                /*** Predict internal tracks from previous frame ***/
                tracker_left_.Predict();

                tracker_left_.Update(detected_line_left);

                tracked_line_left = tracker_left_.GetStateAsBbox();

                // printf("[%s]: tracked_left_k [%f], tracked_left_b: [%f].\n", __func__, tracked_line_left[0], tracked_line_left[1]);
            }
            else
            {
                // printf("[%s]: not_detected_num_left_ [%d].\n", __func__, not_detected_num_left_);

                if (not_detected_num_left_ <= not_tracking_max_)
                {
                    tracker_left_.Predict();
                    tracked_line_left = tracker_left_.GetStateAsBbox();

                    // printf("[%s]: tracked_left_k [%f], tracked_left_b: [%f].\n", __func__, tracked_line_left[0], tracked_line_left[1]);

                    is_detected_left_ = true;
                }
                else
                {
                    is_detected_left_ = false;
                    is_init_tracker_left_ = false;
                }
            }
        }

        if (!is_init_tracker_right_ && is_detected_right_)
        {
            tracker_right_.Init(detected_line_right);
            is_init_tracker_right_ = true;

            tracked_line_right = detected_line_right;

            // printf("[%s]: tracked_right_k [%f], tracked_right_b: [%f].\n", __func__, tracked_line_right[0], tracked_line_right[1]);
        }
        else
        {
            if (is_detected_right_)
            {
                /*** Predict internal tracks from previous frame ***/
                tracker_right_.Predict();

                tracker_right_.Update(detected_line_right);

                tracked_line_right = tracker_right_.GetStateAsBbox();

                // printf("[%s]: tracked_right_k [%f], tracked_right_b: [%f].\n", __func__, tracked_line_right[0], tracked_line_right[1]);
            }
            else
            {
                // printf("[%s]: not_detected_num_right_ [%d].\n", __func__, not_detected_num_right_);

                if (not_detected_num_right_ <= not_tracking_max_)
                {
                    tracker_right_.Predict();
                    tracked_line_right = tracker_right_.GetStateAsBbox();

                    // printf("[%s]: tracked_right_k [%f], tracked_right_b: [%f].\n", __func__, tracked_line_right[0], tracked_line_right[1]);

                    is_detected_right_ = true;
                }
                else
                {
                    is_detected_right_ = false;
                    is_init_tracker_right_ = false;
                }
            }
        }
    }

    void CurbDetect::GetPointsWithRoadside(const PointCloudPtr& pc_original,
                                           PointCloudPtr& pc_on_road,
                                           Eigen::Vector2d& tracked_line_left,
                                           Eigen::Vector2d& tracked_line_right)
    {
        if (is_detected_left_ && tracked_line_left[1] > 1.0 && is_detected_right_ && tracked_line_right[1] < -1.0)
        {
            for (int i = 0; i < pc_original->size(); i++)
            {
                double in_left = (*pc_original)[i].data_[0] * tracked_line_left[0] + tracked_line_left[1] - (*pc_original)[i].data_[1];
                double in_right = (*pc_original)[i].data_[0] * tracked_line_right[0] + tracked_line_right[1] - (*pc_original)[i].data_[1];
                if (in_left > 0 && in_right < 0)
                {
                    (*pc_on_road).push_back((*pc_original)[i]);
                }
            }
        }
        else if (is_detected_left_ && tracked_line_left[1] > 1.0)
        {
            for (int i = 0; i < pc_original->size(); i++)
            {
                double in_left = (*pc_original)[i].data_[0] * tracked_line_left[0] + tracked_line_left[1] - (*pc_original)[i].data_[1];
                if (in_left > 0)
                {
                    (*pc_on_road).push_back((*pc_original)[i]);
                }
            }
        }
        else if (is_detected_right_ && tracked_line_right[1] < -1.0)
        {
            for (int i = 0; i < pc_original->size(); i++)
            {
                double in_right = (*pc_original)[i].data_[0] * tracked_line_right[0] + tracked_line_right[1] - (*pc_original)[i].data_[1];
                if (in_right < 0)
                {
                    (*pc_on_road).push_back((*pc_original)[i]);
                }
            }
        }
        else
        {
            *pc_on_road = *pc_original;
        }

        // printf("[%s]: pc_on_road has [%d] points.\n", __func__, pc_on_road->size());
    }
}  // namespace lidar_perception_ros
