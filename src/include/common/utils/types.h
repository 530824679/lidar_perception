//
// Created by chenwei on 20-5-28.
//

#ifndef _LIDAR_PERCEPTION_ROS_TYPES_H_
#define _LIDAR_PERCEPTION_ROS_TYPES_H_

namespace lidar_perception_ros{

    // calibrate param
    struct CalibrateParam{
        double roll_ = 0.0;
        double pitch_ = 0.0;
        double yaw_ = 0.0;
        double tx_ = 0.0;
        double ty_ = 0.0;
        double tz_ = 0.0;
    };

    // roi param
    struct ROIParam{
        int roi_x_min_ = 0;
        int roi_x_max_ = 0;
        int roi_y_min_ = 0;
        int roi_y_max_ = 0;
        float roi_z_min_ = 0.0;
        float roi_z_max_ = 0.0;
    };

    // voxel param
    struct VoxelParam{
        float voxel_x_ = 0.0;
        float voxel_y_ = 0.0;
        float voxel_z_ = 0.0;
    };

    // Segment param
    struct SegmentParam{
        int row_ = 0;
        int column_ = 0;
        int max_iterations_ = 0;
        float distance_tolerate_ = 0.0;
        float grid_size_ = 0.0;
        float height_threshold_ = 0.0;
    };

    // Curb param
    struct CurbParam{
        int grid_column_ = 0;
        int grid_row_ = 0;
        int min_curb_points_ = 0;
        float min_grid_z_ = 0.0;
        float max_grid_z_ = 0.0;
        float min_ground_plane_z_ = 0.0;
        float max_ground_plane_z_ = 0.0;
        float min_ground_plane_i_ = 0.0;
        float max_ground_plane_i_ = 0.0;
        float grid_size_ = 0.0;
        float height_max_upper_threshold_ = 0.0;
        float height_max_lower_threshold_ = 0.0;
        float height_diff_upper_threshold_ = 0.0;
        float height_diff_lower_threshold_ = 0.0;

        int max_iterations_ = 0;
        int min_ground_plane_points_ = 0;
        int points_threshold_ = 0;
        int min_single_curb_points_ = 0;
        int not_tracking_max_ = 0;
        float plane_dist_threshold_ = 0.0;
        float line_dist_threshold_ = 0.0;
        float angle_threshold_ = 0.0;
    };

    // Cluster param
    struct ClusterParam{
        int min_cluster_size_ = 0;
        int max_cluster_size_ = 0;
        float cluster_distance_ = 0.0;
    };

    // BBox param
    struct BBoxParam{
        float min_distance_ = 0.0;
        float max_distance_ = 0.0;
        float box_min_bottom_ = 0.0;
        float box_min_top_ = 0.0;
        float box_min_volume_ = 0.0;
        float box_max_volume_ = 0.0;
        float height_threshold_ = 0.0;
    };

    // Tracker param
    struct TrackerParam{
        int max_coast_cycles_ = 0;
        int min_hits_ = 0;
        int lidar_rate_ = 0;
        int acceleration_threshold_ = 0;
        float min_confidence_ = 0;
        float filter_threshold_=0;
    };

    struct TrackerInfo{
        int id = 0;

        float center_x = 0.0;
        float center_y = 0.0;
        float center_z = 0.0;

        float width = 0.0;
        float length = 0.0;
        float height = 0.0;

        float yaw = 0.0;

        float velocity_x = 0.0;
        float velocity_y = 0.0;

        float pre_velocity_x = 0.0;
        float pre_velocity_y = 0.0;

        float sum_velocity_x = 0.0;//the sum of velocity during five frames
        float sum_velocity_y = 0.0;

        float pre_center_x = 0.0;
        float pre_center_y = 0.0;

        int year = 0;//the age of tracker
    };

    enum GridCellType{
        UNKNOW = 0,
        GROUND = 1,
        CRUB = 2,
        OBSTACLE = 3
    };

    enum Category{
        UNKNOWN = 0,
        PEDESTRIAN = 1,
        MOTOR = 2,
        CAR = 3,
        TRUCK = 4
    };

    enum Moion{
        MOTION_UNKNOWN = 0,
        MOTION_MOVING = 1,
        MOTION_STATIONARY = 2
    };

    struct Color{
    public:
        Color(float r, float g, float b):r_(r), g_(g), b_(b){};
        float GetR(){ return r_; };
        float GetG(){ return g_; };
        float GetB(){ return b_; };

    private:
        float r_;
        float g_;
        float b_;
    };

    struct BBox{
        // center point
        float x;
        float y;
        float z;

        // dimension
        float dx;
        float dy;
        float dz;

        // angle
        float yaw;

        // centroid point
        float centroid_x;
        float centroid_y;
        float centroid_z;
    };
}

#endif //_LIDAR_PERCEPTION_ROS_TYPES_H_
