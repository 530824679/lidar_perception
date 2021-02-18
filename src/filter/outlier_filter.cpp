#include "filter/outlier_filter.h"
#include <iostream>

namespace lidar_perception_ros{

    void OutlierFilter::InvalidRemove(PointCloudPtr& input_cloud_ptr)
    {
        for (auto it = (*input_cloud_ptr).begin(); it != (*input_cloud_ptr).end();)
        {
            //cout << (*it).GetX() << "   "<< (*it).GetY() << "   "<< (*it).GetZ() << std::endl;
            if (((*it).GetX() == 0.0) && ((*it).GetY() == 0.0) && ((*it).GetZ() == 0.0))
            {
                (*input_cloud_ptr).erase(it++);
            }
            else
            {
                it++;
            }
        }
    }

    void OutlierFilter::OutlierRemove(const PointCloudPtr& input_cloud_ptr, PointCloudPtr& out_cloud_ptr)
    {
        // @name:    OutlierRemove
        // @summary: remove the outlier point
        // @input:   input_cloud_ptr
        // @param:
        // @return:  out_cloud_ptr
        // TMP
        for (int k = 0; k < (*input_cloud_ptr).size(); ++k)
        {
            (*out_cloud_ptr).push_back((*input_cloud_ptr)[k]);
        }

        InvalidRemove(out_cloud_ptr);

        // To do debug
        //        double average_x = 0;
        //        double average_y = 0;
        //        double average_z = 0;
        //
        //        double variance_x = 0;
        //        double variance_y = 0;
        //        double variance_z = 0;
        //
        //        for (int i = 0; i < input_point_cloud.size(); ++i)
        //        {
        //            average_x += input_point_cloud[i].GetX();
        //            average_y += input_point_cloud[i].GetY();
        //            average_z += input_point_cloud[i].GetZ();
        //        }
        //
        //        average_x /= input_point_cloud.size();
        //        average_y /= input_point_cloud.size();
        //        average_z /= input_point_cloud.size();
        //
        //        for (int j = 0; j < input_point_cloud.size(); ++j)
        //        {
        //            variance_x += (input_point_cloud[j].GetX() - average_x) * (input_point_cloud[j].GetX() - average_x);
        //            variance_y += (input_point_cloud[j].GetY() - average_y) * (input_point_cloud[j].GetY() - average_y);
        //            variance_z += (input_point_cloud[j].GetZ() - average_z) * (input_point_cloud[j].GetZ() - average_z);
        //        }
        //        variance_x /= input_point_cloud.size();
        //        variance_y /= input_point_cloud.size();
        //        variance_z /= input_point_cloud.size();
        //
        //        for (int k = 0; k < input_point_cloud.size(); ++k)
        //        {
        //            if(((abs(input_point_cloud[k].GetZ() - average_z) / sqrt(variance_z)) < 1.5 || (abs(input_point_cloud[k].GetZ() + average_z) / sqrt(variance_z)) < 1.5)
        //               && ((abs(input_point_cloud[k].GetY() - average_y) / sqrt(variance_y)) < 1.8 || (abs(input_point_cloud[k].GetY() + average_y) / sqrt(variance_y)) < 1.5)
        //               && ((abs(input_point_cloud[k].GetX() - average_x) / sqrt(variance_x)) < 1.8 || (abs(input_point_cloud[k].GetX() + average_x) / sqrt(variance_x)) < 1.5))
        //            {
        //                out_point_cloud.push_back(input_point_cloud[k]);
        //            }
        //        }
        return;
    }
}

