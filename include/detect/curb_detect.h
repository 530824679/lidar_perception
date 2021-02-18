#ifndef _LIDAR_PERCEPTION_ROS_CURB_DETECT_H_
#define _LIDAR_PERCEPTION_ROS_CURB_DETECT_H_

// system include
#include <math.h>
#include <memory>

// local include
#include "filter/roi_filter.h"
#include "tracker.h"
#include "grid.h"

#include "common/utils/point3d.h"
#include "common/utils/types.h"

namespace lidar_perception_ros
{
    class CurbDetect
    {
    protected:
        typedef typename std::vector<PointXYZI<float>> PointCloud;
        typedef typename std::shared_ptr<std::vector<PointXYZI<float>>> PointCloudPtr;

    public:
        CurbDetect() = default;
        CurbDetect(ROIParam roi_param, CurbParam curb_param);
        ~CurbDetect();

        void Detect(const PointCloudPtr &input_cloud_ptr, PointCloudPtr &out_cloud_ptr, Eigen::Vector4d &plane_coefficients);

    private:
        // Ground plane segmentation
        bool GetGroundPlaneCoeffs(const PointCloudPtr& input_cloud_ptr,
                                  Eigen::Vector4d& plane_coefficients);

        // Get roadside points
        bool GetCurbPoints(const PointCloudPtr& input_cloud_ptr,
                               Eigen::Vector4d& plane_coefficients,
                               PointCloudPtr& curb_cloud_ptr);

        // Separate points to fit lines
        void SeparatePointsToFit(const PointCloudPtr& curb_cloud_ptr,
                                 PointCloudPtr& curb_cloud_left_ptr,
                                 PointCloudPtr& curb_cloud_right_ptr);

        // Line model
        void FitLineRansac(const PointCloudPtr& curb_cloud_left_ptr,
                           const PointCloudPtr& curb_cloud_right_ptr,
                           Eigen::Vector2d &detected_line_left,
                           Eigen::Vector2d &detected_line_right);

        void LineTracking(Eigen::Vector2d &detected_line_left,
                          Eigen::Vector2d &detected_line_right,
                          Eigen::Vector2d &tracked_line_left,
                          Eigen::Vector2d &tracked_line_right);

        void GetPointsWithRoadside(const PointCloudPtr& pc_original,
                                   PointCloudPtr& pc_on_road,
                                   Eigen::Vector2d &tracked_line_left,
                                   Eigen::Vector2d &tracked_line_right);

        bool IsContinueAdd(vector<float> &input, int n);
        bool IsContinueReduce(vector<float> &input, int n);

        enum MotionType
        {
            Straight,
            TurnLeft,
            TurnRight
        };

        // For detect
        int grid_row_;
        int grid_col_;
        int min_curb_points_;
        float grid_size_;
        float min_x_;
        float max_x_;
        float min_y_;
        float max_y_;
        float min_z_;
        float max_z_;
        float min_grid_z_;
        float max_grid_z_;
        float min_ground_plane_z_;
        float max_ground_plane_z_;
        float max_ground_plane_x_;
        float min_ground_plane_i_;
        float max_ground_plane_i_;
        float height_max_upper_threshold_;
        float height_max_lower_threshold_;
        float height_diff_upper_threshold_;
        float height_diff_lower_threshold_;

        float route_threshold_;
        MotionType ego_motion;

        // For fit
        int max_iterations_;
        int min_ground_plane_points_;
        int points_threshold_left_;
        int points_threshold_right_;
        int min_left_curb_points_;
        int min_right_curb_points_;
        float plane_dist_threshold_;
        float line_dist_threshold_;
        float angle_threshold_;

        // For tracking
        Tracker tracker_left_, tracker_right_;
        bool is_detected_left_;
        bool is_detected_right_;
        bool is_init_tracker_left_;
        bool is_init_tracker_right_;
        int not_detected_num_left_;
        int not_detected_num_right_;
        int not_tracking_max_;

    private:
        bool is_curb_pts_;
        bool is_get_ground_plane_;

    };

}  // namespace lidar_perception_ros

#endif  // _LIDAR_PERCEPTION_ROS_CURB_DETECT_H_
