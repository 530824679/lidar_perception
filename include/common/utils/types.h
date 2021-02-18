//
// Created by chenwei on 20-5-28.
//

#ifndef _LIDAR_PERCEPTION_ROS_TYPES_H_
#define _LIDAR_PERCEPTION_ROS_TYPES_H_

#include <string>
#include <vector>
#include <math.h>

namespace lidar_perception_ros
{
// calibrate param
    struct CalibrateParam
    {
        double roll_ = 0.0;
        double pitch_ = 0.0;
        double yaw_ = 0.0;
        double tx_ = 0.0;
        double ty_ = 0.0;
        double tz_ = 0.0;
    };

// roi param
    struct ROIParam
    {
        float roi_x_min_ = 0.0;
        float roi_x_max_ = 0.0;
        float roi_y_min_ = 0.0;
        float roi_y_max_ = 0.0;
        float roi_z_min_ = 0.0;
        float roi_z_max_ = 0.0;
    };

// voxel param
    struct VoxelParam
    {
        float voxel_x_ = 0.0;
        float voxel_y_ = 0.0;
        float voxel_z_ = 0.0;
    };

// Segment param
    struct SegmentParam
    {
        int row_ = 0;
        int column_ = 0;
        int max_iterations_ = 0;
        float distance_tolerate_ = 0.0;
        float grid_size_ = 0.0;
        float height_threshold_ = 0.0;
    };

// Curb param
    struct CurbParam
    {
        int grid_column_ = 0;
        int grid_row_ = 0;
        int min_curb_points_ = 0;
        float min_grid_z_ = 0.0;
        float max_grid_z_ = 0.0;
        float min_ground_plane_z_ = 0.0;
        float max_ground_plane_z_ = 0.0;
        float max_ground_plane_x_ = 0.0;
        float min_ground_plane_i_ = 0.0;
        float max_ground_plane_i_ = 0.0;
        float grid_size_ = 0.0;
        float height_max_upper_threshold_ = 0.0;
        float height_max_lower_threshold_ = 0.0;
        float height_diff_upper_threshold_ = 0.0;
        float height_diff_lower_threshold_ = 0.0;
        float route_threshold_ = 0.0;

        int max_iterations_ = 0;
        int min_ground_plane_points_ = 0;
        int points_threshold_left_ = 0;
        int points_threshold_right_ = 0;
        int min_left_curb_points_ = 0;
        int min_right_curb_points_ = 0;
        int not_tracking_max_ = 0;
        float plane_dist_threshold_ = 0.0;
        float line_dist_threshold_ = 0.0;
        float angle_threshold_ = 0.0;
    };

// Cluster param
    struct ClusterParam
    {
        int min_cluster_size_ = 0;
        int max_cluster_size_ = 0;
        float cluster_distance_ = 0.0;
        int dbscan_min_points_ = 0;
        float dbscan_tolerance_ = 0.0;
        bool use_cluster_merge_ = false;
        float cluster_merge_threshold_ = 0.0;
        float cluster_merge_threshold_y_ = 0.0;
        bool use_multi_thresholds_ = false;
        float cluster_ranges_[2] = {0.0};
        float cluster_distances_[3] = {0.0};
        float dbscan_tolerances_[3] = {0.0};
    };

// BBox param
    struct BBoxParam
    {
        float min_distance_ = 0.0;
        float max_distance_ = 0.0;
        float box_min_bottom_ = 0.0;
        float box_min_top_ = 0.0;
        float box_min_area_ = 0.0;
        float box_max_area_ = 0.0;
        float box_min_volume_ = 0.0;
        float box_max_volume_ = 0.0;
        float height_threshold_ = 0.0;
    };

// Tracker param
    struct TrackerParam
    {
        int max_coast_cycles_ = 0;
        int min_hits_ = 0;
        int lidar_rate_ = 0;
        int acceleration_threshold_ = 0;
        float min_confidence_ = 0;
        float filter_threshold_ = 0;
        int kernel_size_ = 0;
        float iou_weight_ = 0.0;
        float rotate_iou_weight_ = 0.0;
        float distance_weight_ = 0.0;
        float centroid_weight_ = 0.0;
        float pointnum_weight_ = 0.0;
        bool use_mutil_threshold_ = true;
        float first_threshold_ = 0.0;
        float second_threshold_ = 0.0;
        float third_threshold_ = 0.0;
        int tracking_distance_[4] = {0};
    };

    struct Point2D{
        Point2D(){};
        Point2D(float a, float b):x(a), y(b){};
        float x;
        float y;
    };

    typedef union suf32
    {
        int i;
        unsigned u;
        float f;
    }
            suf32;


    struct Size2D{
        Size2D(){};
        Size2D(float a,float b):width(a),height(b){};
        float area() const{
            return width*height;
        }
        float width;
        float height;

    };

    class RotatedRect{
    public:
        RotatedRect(){};
        RotatedRect(Point2D a, Size2D b, float c):center(a), size(b), angle(c){};
        void points(Point2D pts[]) const
        {
            double _angle = angle*M_PI/180.;
            float b = (float)cos(_angle)*0.5f;
            float a = (float)sin(_angle)*0.5f;

            pts[0].x = center.x - a*size.height - b*size.width;
            pts[0].y = center.y + b*size.height - a*size.width;
            pts[1].x = center.x + a*size.height - b*size.width;
            pts[1].y = center.y - b*size.height - a*size.width;
            pts[2].x = 2*center.x - pts[0].x;
            pts[2].y = 2*center.y - pts[0].y;
            pts[3].x = 2*center.x - pts[1].x;
            pts[3].y = 2*center.y - pts[1].y;
        };
        Point2D center;
        Size2D size;
        float angle;
    };

    class Rect{
    public:
        Rect(){};
        Rect(float _x, float _y, float _width, float _height):x(_x),y(_y),width(_width),height(_height){};
        // Rect intersection_area (const Rect a, const Rect b);
        // Rect union_area(const Rect a, const Rect b);
        float area(){return width*height;}
        float x;
        float y;
        float width;
        float height;
    };
    struct TrackerInfo
    {
        int id = 0;

        std::string label;
        float score = 0.0;
        bool valid = false;

        //3D BB
        //Pose x,y,z,yaw,roll,pitch
        float center_x = 0.0;
        float center_y = 0.0;
        float center_z = 0.0;
        float yaw = 0.0;
        float pitch = 0.0;
        float roll = 0.0;

        //dimensions
        float width = 0.0;
        float length = 0.0;
        float height = 0.0;

        //centroid
        float centroid_x = 0.0;
        float centroid_y = 0.0;
        float centroid_z = 0.0;

        //velocity
        float velocity_x = 0.0;
        float velocity_y = 0.0;
        float velocity_z = 0.0;

        //acceleration
        float acceleration_x = 0.0;
        float acceleration_y = 0.0;
        float acceleration_z = 0.0;

        float pre_velocity_x = 0.0;
        float pre_velocity_y = 0.0;

        float sum_velocity_x = 0.0;  //the sum of velocity during five frames
        float sum_velocity_y = 0.0;

        float pre_center_x = 0.0;
        float pre_center_y = 0.0;

        int points_num = 0.0;

        int year = 0;  //the age of tracker

        float angle = 0;

        std::vector<Point2D> corners;
    };

    enum GridCellType
    {
        UNKNOW = 0,
        GROUND = 1,
        CRUB = 2,
        OBSTACLE = 3
    };

    enum Category
    {
        CAR = 0,
        BUS = 1,
        PEDESTRIAN = 2,
        UNKNOWN = 3,
    };

    enum Moion
    {
        MOTION_UNKNOWN = 0,
        MOTION_MOVING = 1,
        MOTION_STATIONARY = 2
    };

    struct Color
    {
    public:
        Color(float r, float g, float b) : r_(r), g_(g), b_(b){};
        float GetR() { return r_; };
        float GetG() { return g_; };
        float GetB() { return b_; };

    private:
        float r_;
        float g_;
        float b_;
    };


    struct BBox
    {
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
        float rotate;

        // centroid point
        float centroid_x;
        float centroid_y;
        float centroid_z;

        int points_num;

        float score;
        Category category;

        std::vector<Point2D> vertex_pts;

        //tmp rect
        float rect_x;
        float rect_y;
        float rect_z;
        float rect_dx;
        float rect_dy;
        float rect_dz;

    };
}  // namespace lidar_perception_ros

#endif  //_LIDAR_PERCEPTION_ROS_TYPES_H_
