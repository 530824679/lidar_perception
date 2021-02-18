#include "filter/roi_filter.h"

namespace lidar_perception_ros{

    ROIFilter::ROIFilter()
    {
        filter_limit_min_x_ = FLT_MIN;
        filter_limit_max_x_ = FLT_MAX;
        filter_limit_min_y_ = FLT_MIN;
        filter_limit_max_y_ = FLT_MAX;
        filter_limit_min_z_ = FLT_MIN;
        filter_limit_max_z_ = FLT_MAX;
    }

    ROIFilter::ROIFilter(ROIParam roi_param)
    {
        filter_limit_min_x_ = roi_param.roi_x_min_;
        filter_limit_max_x_ = roi_param.roi_x_max_;
        filter_limit_min_y_ = roi_param.roi_y_min_;
        filter_limit_max_y_ = roi_param.roi_y_max_;
        filter_limit_min_z_ = roi_param.roi_z_min_;
        filter_limit_max_z_ = roi_param.roi_z_max_;
    }

    ROIFilter::~ROIFilter()
    {
        point_cloud_ptr_.reset();
        indices_ptr_.reset();
    }

    void ROIFilter::PassThough(const PointCloudPtr& input_cloud_ptr, PointCloudPtr& output_cloud_ptr)
    {
        SetInputCloud(input_cloud_ptr);
        SetFilterLimits(filter_limit_min_x_, filter_limit_max_x_, filter_limit_min_y_, filter_limit_max_y_, filter_limit_min_z_, filter_limit_max_z_);
        Filter(*output_cloud_ptr);
    }

    bool ROIFilter::InitCompute()
    {
        if(!point_cloud_ptr_)
        {
            return (false);
        }

        if(!indices_ptr_)
        {
            indices_ptr_.reset(new std::vector<int>);
            try
            {
                indices_ptr_->resize(point_cloud_ptr_->size());
            }
            catch (const std::bad_alloc&)
            {
                printf("[%s]: Failed to allocate %lu indices.\n", __func__, point_cloud_ptr_->size());
            }
            for (size_t i = 0; i < indices_ptr_->size(); i++)
            {
                (*indices_ptr_)[i] = static_cast<int>(i);
            }
        }
        else
        {
            printf("[%s]: It's already indices.\n", __func__);
        }

        if(indices_ptr_->size() != point_cloud_ptr_->size())
        {
            size_t indices_size = indices_ptr_->size();
            indices_ptr_->resize(point_cloud_ptr_->size());
            for (size_t i = indices_size; i < indices_ptr_->size(); ++i)
            {
                (*indices_ptr_)[i] = static_cast<int>(i);
            }
        }
        return (true);
    }

    bool ROIFilter::DeinitCompute()
    {
        point_cloud_ptr_.reset();
        indices_ptr_.reset();
        return (true);
    }

    void ROIFilter::SetInputCloud(const PointCloudPtr &cloud)
    {
        point_cloud_ptr_ = cloud;
    }

    void ROIFilter::SetFilterLimits(const float &limit_min_x, const float &limit_max_x, const float &limit_min_y, const float &limit_max_y, const float &limit_min_z, const float &limit_max_z)
    {
        filter_limit_min_x_ = limit_min_x;
        filter_limit_max_x_ = limit_max_x;
        filter_limit_min_y_ = limit_min_y;
        filter_limit_max_y_ = limit_max_y;
        filter_limit_min_z_ = limit_min_z;
        filter_limit_max_z_ = limit_max_z;
    }

    void ROIFilter::Filter(PointCloud &output_cloud)
    {
        if (!InitCompute())
        {
            return;
        }

        std::vector<int> indices;
        indices.resize(indices_ptr_->size());
        int output_indices_iterator = 0;

        for (int i = 0; i < static_cast<int>(indices_ptr_->size()); ++i)
        {

            if ( !std::isfinite((*point_cloud_ptr_)[(*indices_ptr_)[i]].GetX()) ||
                 !std::isfinite((*point_cloud_ptr_)[(*indices_ptr_)[i]].GetY()) ||
                 !std::isfinite((*point_cloud_ptr_)[(*indices_ptr_)[i]].GetZ()))
            {
                continue;
            }

            float valueX = (*point_cloud_ptr_)[(*indices_ptr_)[i]].GetX();
            float valueY = (*point_cloud_ptr_)[(*indices_ptr_)[i]].GetY();
            float valueZ = (*point_cloud_ptr_)[(*indices_ptr_)[i]].GetZ();

            if (!std::isfinite(valueX) || !std::isfinite(valueY) || !std::isfinite(valueZ))
            {
                continue;
            }

            if (valueX < filter_limit_min_x_ || valueX > filter_limit_max_x_ ||
                valueY < filter_limit_min_y_ || valueY > filter_limit_max_y_ ||
                valueZ < filter_limit_min_z_ || valueZ > filter_limit_max_z_)
            {
                continue;
            }

            indices[output_indices_iterator++] = (*indices_ptr_)[i];
        }

        indices.resize(output_indices_iterator);

        CopyPointCloud(*point_cloud_ptr_, indices, output_cloud);

        DeinitCompute();
    }

    void ROIFilter::CopyPointCloud(const PointCloud &input_cloud, const std::vector<int> &indices, PointCloud &output_cloud)
    {
        if (!output_cloud.empty())
            output_cloud.clear();

        if (indices.size() == input_cloud.size())
        {
            output_cloud = input_cloud;
            return;
        }

        for (size_t i = 0; i < indices.size(); ++i) {
            output_cloud.push_back(input_cloud[indices[i]]);
        }
    }

}

