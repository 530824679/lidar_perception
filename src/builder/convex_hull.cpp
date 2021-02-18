#include "builder/convex_hull.h"

namespace lidar_perception_ros
{

    PointXYZI<float> origin_;

    double CrossProduct(PointXYZI<float> pt0, PointXYZI<float> pt1, PointXYZI<float> pt2)
    {
        return (pt1.GetX() - pt0.GetX())*(pt2.GetY() - pt0.GetY()) - (pt2.GetX() - pt0.GetX())*(pt1.GetY() - pt0.GetY());
    }

    double Distance(PointXYZI<float> pt1, PointXYZI<float> pt2)
    {
        return sqrt((pt1.GetX() - pt2.GetX())*(pt1.GetX() - pt2.GetX()) + (pt1.GetY() - pt2.GetY())*(pt1.GetY() - pt2.GetY()));
    }

    void ConvexHull::GrahamScan(PointCloudPtr& clusters_ptr, PointCloud& hull)
    {
        int top = 2;
        int index = 0;
        for (int i = 1; i < (*clusters_ptr).size(); i++)
        {
            if((*clusters_ptr)[i].GetY() < (*clusters_ptr)[index].GetY() || ((*clusters_ptr)[i].GetY() == (*clusters_ptr)[index].GetY() && (*clusters_ptr)[i].GetX() < (*clusters_ptr)[index].GetX()))
            {
                index = i;
            }
        }
        swap((*clusters_ptr)[0], (*clusters_ptr)[index]);
        hull.push_back((*clusters_ptr)[0]);

        origin_ = (*clusters_ptr)[0];
        std::sort((*clusters_ptr).begin()+1, (*clusters_ptr).end()-1, [](PointXYZI<float> pt1, PointXYZI<float> pt2){double tmp = CrossProduct(origin_, pt1, pt2); if(fabs(tmp) < 1e-6) return Distance(origin_, pt1) < Distance(origin_, pt2);else return tmp > 0;});
        hull.push_back((*clusters_ptr)[1]);
        hull.push_back((*clusters_ptr)[2]);

        for (int i = 3; i < (*clusters_ptr).size(); ++i)
        {
            while (top > 0 && CrossProduct(hull[top - 1], (*clusters_ptr)[i], hull[top]) >= 0)
            {
                --top;
                hull.pop_back();
            }
            hull.push_back((*clusters_ptr)[i]);
            ++top;
        }

    }

    void ConvexHull::GetAngle(PointCloud &hull, vector<float> &angle)
    {
        float tmp = 0;
        for (int i = 0; i < hull.size(); ++i) {
            tmp = atan2(hull[i+1].GetY() - hull[i].GetY(), hull[i+1].GetX() - hull[i].GetX());
            if (tmp < 0)
                tmp += M_PI / 2;
            angle[i] = -tmp;
        }
    }

    void ConvexHull::UniqueAngle(vector<float> &angle, vector<unsigned int> &state)
    {
        state.resize(angle.size(), 1);
        for (int i = 0; i < angle.size(); i++)
        {
            for (int j = i + 1; j < angle.size(); j++)
            {
                if (angle[i] == angle[j])
                {
                    state[j] = 0;
                }
            }
        }

    }

    void ConvexHull::GetRotatePoints(PointCloud &hull, float theta, vector<Point2D> &rotate_points)
    {
        for (int i = 0; i < hull.size(); ++i)
        {
            Point2D pt(0, 0);
            pt.x = hull[i].GetX() * cos(theta) - hull[i].GetY() * sin(theta);
            pt.y = hull[i].GetX() * sin(theta) + hull[i].GetY() * cos(theta);
            rotate_points.push_back(pt);
        }
    }


    float ConvexHull::GetDistance(const Point2D p1, const Point2D p2)
    {
        return sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }

    void ConvexHull::GetRemapPoints(vector<Point2D> &rotate_points, float theta, vector<Point2D> &remap_points)
    {
        if (rotate_points.size() != 4)
        {
            std::cerr << "rotate point size error." << std::endl;
            return;
        }

        for (int i = 0; i < rotate_points.size(); i++)
        {
            remap_points[i].x = rotate_points[i].x * cos(theta) + rotate_points[i].y * sin(theta);
            remap_points[i].y = -rotate_points[i].x * sin(theta) + rotate_points[i].y * cos(theta);
        }
    }

    void ConvexHull::MinAreaRect(PointCloud &hull, BBox &box)
    {
        int hull_num = hull.size();

        vector<unsigned int> state;
        vector<float> angle(hull_num, 0.0);
        vector<Point2D> rect_pt(4);
        vector<Point2D> remap_pt(4);

        GetAngle(hull, angle);
        UniqueAngle(angle, state);

        float min_area = FLT_MAX;
        float min_angle = 0.;
        for (int i = 0; i < hull_num; ++i)
        {
            float area = 0.0;
            float max_x = -FLT_MAX;
            float min_x = FLT_MAX;
            float max_y = -FLT_MAX;
            float min_y = FLT_MAX;
            vector<Point2D> rotate_points;

            if (state[i] == 1)
            {
                GetRotatePoints(hull, angle[i], rotate_points);
                for (int j = 0; j < hull_num; ++j)
                {
                    if (rotate_points[j].x > max_x)
                        max_x = rotate_points[j].x;
                    if (rotate_points[j].x < min_x)
                        min_x = rotate_points[j].x;
                    if (rotate_points[j].y > max_y)
                        max_y = rotate_points[j].y;
                    if (rotate_points[j].y < min_y)
                        min_y = rotate_points[j].y;
                }
                area = ((max_x - min_x) * (max_y - min_y));
                if (area < min_area)
                {
                    min_area = area;
                    min_angle = angle[i];
                    rect_pt[0].x = min_x;
                    rect_pt[0].y = min_y;
                    rect_pt[1].x = min_x;
                    rect_pt[1].y = max_y;
                    rect_pt[2].x = max_x;
                    rect_pt[2].y = min_y;
                    rect_pt[3].x = max_x;
                    rect_pt[3].y = max_y;
                }
            }
        }

        GetRemapPoints(rect_pt, min_angle, remap_pt);

        box.x = (remap_pt[0].x + remap_pt[3].x) / 2.0;
        box.y = (remap_pt[0].y + remap_pt[3].y) / 2.0;
        constexpr float ep = 0.001;
        box.dx = std::max(GetDistance(rect_pt[0],rect_pt[2]), ep);
        box.dy = std::max(GetDistance(rect_pt[0],rect_pt[1]), ep);
        box.yaw = min_angle;
    }
}