#include "filter/roi_filter.h"

namespace lidar_perception_ros{

    ROIFilter::ROIFilter()
    {
        //point_cloud_ptr_ = std::make_shared<PointCloud>();
        filter_field_name_ = "";
        filter_limit_min_ = FLT_MIN;
        filter_limit_max_ = FLT_MAX;
    }

    ROIFilter::~ROIFilter()
    {
        point_cloud_ptr_.reset();
        indices_ptr_.reset();
    }

    void ROIFilter::PassThough(const PointCloudPtr& input_cloud_ptr, PointCloudPtr& output_cloud_ptr, ROIParam roi_param)
    {
        SetInputCloud(input_cloud_ptr);
        SetFilterFieldName("x");
        SetFilterLimits(roi_param.roi_x_min_, roi_param.roi_x_max_);
        Filter(*output_cloud_ptr);

        SetInputCloud(output_cloud_ptr);
        SetFilterFieldName("y");
        SetFilterLimits(roi_param.roi_y_min_, roi_param.roi_y_max_);
        Filter(*output_cloud_ptr);

        SetInputCloud(output_cloud_ptr);
        SetFilterFieldName("z");
        SetFilterLimits(roi_param.roi_z_min_, roi_param.roi_z_max_);
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

    void ROIFilter::SetFilterFieldName(const std::string &field_name)
    {
        filter_field_name_ = field_name;
    }

    void ROIFilter::SetFilterLimits(const float &limit_min, const float &limit_max)
    {
        filter_limit_min_ = limit_min;
        filter_limit_max_ = limit_max;
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

            float value = 0;
            if (filter_field_name_ == "x") {
                value = (*point_cloud_ptr_)[(*indices_ptr_)[i]].GetX();
            }else if (filter_field_name_ == "y") {
                value = (*point_cloud_ptr_)[(*indices_ptr_)[i]].GetY();
            }else if (filter_field_name_ == "z") {
                value = (*point_cloud_ptr_)[(*indices_ptr_)[i]].GetZ();
            } else {
                printf("[%s]: Unable to find field name in point type.\n", __func__);
                indices.clear();
                return;
            }

            if (!std::isfinite(value))
            {
                continue;
            }

            if (value < filter_limit_min_ || value > filter_limit_max_)
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

