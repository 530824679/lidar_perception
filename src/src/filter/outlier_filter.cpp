#include "filter/outlier_filter.h"

namespace lidar_perception_ros{
    
    void OutlierFilter::OutlierRemove(PointCloud& input_point_cloud, PointCloud& out_point_cloud)
    {
        double average_x = 0;
        double average_y = 0;
        double average_z = 0;

        double variance_x = 0;
        double variance_y = 0;
        double variance_z = 0;

        for (int i = 0; i < input_point_cloud.size(); ++i)
        {
            average_x += input_point_cloud[i].GetX();
            average_y += input_point_cloud[i].GetY();
            average_z += input_point_cloud[i].GetZ();
        }

        average_x /= input_point_cloud.size();
        average_y /= input_point_cloud.size();
        average_z /= input_point_cloud.size();

        for (int j = 0; j < input_point_cloud.size(); ++j)
        {
            variance_x += (input_point_cloud[j].GetX() - average_x) * (input_point_cloud[j].GetX() - average_x);
            variance_y += (input_point_cloud[j].GetY() - average_y) * (input_point_cloud[j].GetY() - average_y);
            variance_z += (input_point_cloud[j].GetZ() - average_z) * (input_point_cloud[j].GetZ() - average_z);
        }
        variance_x /= input_point_cloud.size();
        variance_y /= input_point_cloud.size();
        variance_z /= input_point_cloud.size();

        for (int k = 0; k < input_point_cloud.size(); ++k)
        {
            if(((abs(input_point_cloud[k].GetZ() - average_z) / sqrt(variance_z)) < 1.5 || (abs(input_point_cloud[k].GetZ() + average_z) / sqrt(variance_z)) < 1.5)
               && ((abs(input_point_cloud[k].GetY() - average_y) / sqrt(variance_y)) < 1.8 || (abs(input_point_cloud[k].GetY() + average_y) / sqrt(variance_y)) < 1.5)
               && ((abs(input_point_cloud[k].GetX() - average_x) / sqrt(variance_x)) < 1.8 || (abs(input_point_cloud[k].GetX() + average_x) / sqrt(variance_x)) < 1.5))
            {
                out_point_cloud.push_back(input_point_cloud[k]);
            }
        }
        return;
    }
}

