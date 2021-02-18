
#include "segment/object_segment.h"

namespace lidar_perception_ros
{
Segment::Segment(SegmentParam segment_param, ROIParam roi_param)
{
    max_iterations_ = segment_param.max_iterations_;
    distance_tolerate_ = segment_param.distance_tolerate_;
    grid_size_ = segment_param.grid_size_;
    height_threshold_ = segment_param.height_threshold_;
    row_ = int((roi_param.roi_x_max_ - roi_param.roi_x_min_) / grid_size_);
    column_ = int((roi_param.roi_y_max_ - roi_param.roi_y_min_) / grid_size_);
}

Segment::~Segment()
{
}

bool Segment::Ransac3D(const PointCloud& input_cloud, Eigen::Vector4d& plane_coefficients)
{
    PointCloud filter_cloud;

    for (int i = 0; i < input_cloud.size(); i++)
    {
        if (input_cloud[i].GetZ() > -0.5 && input_cloud[i].GetZ() < 0.5)
        {
            filter_cloud.push_back(input_cloud[i]);
        }
    }

    int points_num = filter_cloud.size();

    int index_1 = 0;
    int index_2 = 0;
    int index_3 = 0;

    int inliers_max = 0;

    for (int k = 0; k < max_iterations_;)
    {
        int inliers = 0;

        index_1 = rand() % points_num;
        index_2 = rand() % points_num;
        index_3 = rand() % points_num;

        Point3D pt1, pt2, pt3;
        pt1.x = filter_cloud[index_1].GetX();
        pt1.y = filter_cloud[index_1].GetY();
        pt1.z = filter_cloud[index_1].GetZ();
        pt2.x = filter_cloud[index_2].GetX();
        pt2.y = filter_cloud[index_2].GetY();
        pt2.z = filter_cloud[index_2].GetZ();
        pt3.x = filter_cloud[index_3].GetX();
        pt3.y = filter_cloud[index_3].GetY();
        pt3.z = filter_cloud[index_3].GetZ();

        if (CollineationJudge(pt1, pt2, pt3))
            continue;

        float a, b, c, d, sqrt_abc;
        a = (pt2.y - pt1.y) * (pt3.z - pt1.z) - (pt2.z - pt1.z) * (pt3.y - pt1.y);
        b = (pt2.z - pt1.z) * (pt3.x - pt1.x) - (pt2.x - pt1.x) * (pt3.z - pt1.z);
        c = (pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt2.y - pt1.y) * (pt3.x - pt1.x);
        d = -(a * pt1.x + b * pt1.y + c * pt1.z);
        sqrt_abc = sqrt(a * a + b * b + c * c);

        for (int index = 0; index < points_num; index++)
        {
            PointXYZI<float> point = filter_cloud[index];
            float x = point.GetX();
            float y = point.GetY();
            float z = point.GetZ();
            float dist = fabs(a * x + b * y + c * z + d) / sqrt_abc;

            if (dist < distance_tolerate_)
            {
                inliers++;
            }
        }

        if (inliers > inliers_max)
        {
            inliers_max = inliers;
            plane_coefficients << a, b, c, d;
        }

        k++;
    }

    if (inliers_max > 500)
    {
        std::cerr << "plane_coefficients: " << plane_coefficients << std::endl;
        return true;
    }
    else
    {
        printf("Too few points to estimate ground plane.\n");
        return false;
    }
}

void Segment::ObjectSegment(const PointCloud& input_cloud, PointCloud& out_cloud, Eigen::Vector4d& plane_coefficients)
{
    for (int i = 0; i < input_cloud.size(); ++i)
    {
        PointXYZI<float> pt = input_cloud[i];

        float a = plane_coefficients[0];
        float b = plane_coefficients[1];
        float c = plane_coefficients[2];
        float d = plane_coefficients[3];
        float sqrt_abc = sqrt(a * a + b * b + c * c);
        float dist = fabs(a * pt.GetX() + b * pt.GetY() + c * pt.GetZ() + d) / sqrt_abc;

        if (dist < distance_tolerate_)
        {
            out_cloud.push_back(pt);
        }
    }
}

float Segment::Distance(Point3D& pt1, Point3D& pt2)
{
    float x = pt1.x - pt2.x;
    float y = pt1.y - pt2.y;
    float z = pt1.z - pt2.z;
    return sqrt(x * x + y * y + z * z);
}

bool Segment::CollineationJudge(Point3D& pt1, Point3D& pt2, Point3D& pt3)
{
    float edge_a = Distance(pt1, pt2);
    float edge_b = Distance(pt2, pt3);
    float edge_c = Distance(pt1, pt3);

    float p = 0.5 * (edge_a + edge_b + edge_c);
    float area = p * (p - edge_a) * (p - edge_b) * (p - edge_c);

    if (abs(area) < 1e-6)
        return true;
    else
        return false;
}

float Segment::Min(float x, float y)
{
    return (x) < (y) ? (x) : (y);
}

float Segment::Max(float x, float y)
{
    return (x) > (y) ? (x) : (y);
}

void Segment::CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after, Eigen::Matrix3f& rotate_matrix)
{
    before.normalize();
    after.normalize();

    float angle = acos(before.dot(after));
    Eigen::Vector3f p_rotate = before.cross(after);
    p_rotate.normalize();

    rotate_matrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotate_matrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));
    rotate_matrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

    rotate_matrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotate_matrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotate_matrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

    rotate_matrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotate_matrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotate_matrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
}

void Segment::CalcRotateMatrix(Eigen::Vector4d plane_coefficients, Eigen::Matrix3f& rotate_matrix)
{
    if (plane_coefficients[2] < 0)
    {
        plane_coefficients[0] = 0 - plane_coefficients[0];
        plane_coefficients[1] = 0 - plane_coefficients[1];
        plane_coefficients[2] = 0 - plane_coefficients[2];
    }
    Eigen::Vector3f before(plane_coefficients[0], plane_coefficients[1], plane_coefficients[2]);
    Eigen::Vector3f after(0.f, 0.f, 1.f);
    CreateRotateMatrix(before, after, rotate_matrix);
}

void Segment::CoordTransfor(const PointCloud& input_cloud, Eigen::Matrix3f& rotation_matrix, PointCloud& out_cloud)
{
    for (size_t i = 0; i < input_cloud.size(); i++)
    {
        PointXYZI<float> pt = input_cloud[i];
        PointXYZI<float> out;
        out.SetX(static_cast<float>(rotation_matrix(0, 0) * pt.GetX() + rotation_matrix(0, 1) * pt.GetY() + rotation_matrix(0, 2) * pt.GetZ()));
        out.SetY(static_cast<float>(rotation_matrix(1, 0) * pt.GetX() + rotation_matrix(1, 1) * pt.GetY() + rotation_matrix(1, 2) * pt.GetZ()));
        out.SetZ(static_cast<float>(rotation_matrix(2, 0) * pt.GetX() + rotation_matrix(2, 1) * pt.GetY() + rotation_matrix(2, 2) * pt.GetZ()));
        out.SetI(pt.GetI());
        out_cloud.push_back(out);
    }
}

bool Segment::BuildGridMap(const PointCloud& input_cloud, PointCloud& out_cloud, Eigen::Vector4d plane_coefficients)
{
    // @name:    BuildGridMap
    // @summary: To build grid map
    // @input:   input_cloud, plane_coefficients
    // @param:   grid_size_, row_, column_
    // @return:  out_cloud
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Zero();
    CalcRotateMatrix(plane_coefficients, rotation_matrix);

    PointCloud transform_cloud;
    CoordTransfor(input_cloud, rotation_matrix, transform_cloud);

    PointCloud inverse_transform_cloud;
    Grid grid[row_][column_];
    for (unsigned int count = 0; count < transform_cloud.size(); count++)
    {
        if (std::isnan(transform_cloud[count].GetX()))
            continue;

        int x_grid = floor(double(transform_cloud[count].GetX()) / grid_size_);
        int y_grid = floor(double(transform_cloud[count].GetY()) / grid_size_) + column_ / 2;

        if ((x_grid < row_ && x_grid >= 0) && (y_grid < column_ && y_grid >= 0))
        {
            if (!grid[x_grid][y_grid].init_)
            {
                grid[x_grid][y_grid].min_height_ = transform_cloud[count].GetZ();
                grid[x_grid][y_grid].max_height_ = transform_cloud[count].GetZ();
                grid[x_grid][y_grid].init_ = true;
            }
            else
            {
                grid[x_grid][y_grid].min_height_ = Min(grid[x_grid][y_grid].min_height_, transform_cloud[count].GetZ());
                grid[x_grid][y_grid].max_height_ = Max(grid[x_grid][y_grid].max_height_, transform_cloud[count].GetZ());
            }
            grid[x_grid][y_grid].points_num_++;
            grid[x_grid][y_grid].grid_cloud_.push_back(transform_cloud[count]);
        }
    }

    for (int i = 0; i < row_; ++i)
    {
        for (int j = 0; j < column_; ++j)
        {
            if (!grid[i][j].init_)
            {
                continue;
            }

            for (int k = 0; k < grid[i][j].points_num_; ++k)
            {
                if ((grid[i][j].grid_cloud_[k].GetZ() > height_threshold_))
                {
                    out_cloud.push_back(grid[i][j].grid_cloud_[k]);
                }
            }
        }
    }

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3, 3);
    Eigen::Matrix3f inverse_matrix = rotation_matrix.ldlt().solve(I);
    //Eigen::Matrix3f inverse_matrix = rotation_matrix.inverse();
    CoordTransfor(inverse_transform_cloud, inverse_matrix, out_cloud);
    return true;
}
}  // namespace lidar_perception_ros
