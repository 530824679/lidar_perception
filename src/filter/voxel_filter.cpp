#include "filter/voxel_filter.h"

namespace lidar_perception_ros{

    float VoxelFilter::EucliDistance(PointXYZI<float> &point1, PointXYZI<float> &point2){
        float distance_x = (point1.GetX() - point2.GetX()) * (point1.GetX() - point2.GetX());
        float distance_y = (point1.GetY() - point2.GetY()) * (point1.GetY() - point2.GetY());
        float distance_z = (point1.GetZ() - point2.GetZ()) * (point1.GetZ() - point2.GetZ());
        double distance = sqrt(distance_x + distance_y + distance_z);
        return distance;
    }

    void VoxelFilter::VoxelProcess(PointCloud& input_point_cloud, PointCloud& out_point_cloud, VoxelParam voxel_param){
        if (input_point_cloud.size() == 0){
            printf("[%s]: Input Point Cloud is empty.\n", __func__);
            return;
        }

        Eigen::Vector3f min_p, max_p;
        GetMaxMin(input_point_cloud, min_p, max_p);

        Eigen::Vector3f inverse_leaf_size;
        inverse_leaf_size << 1/voxel_param.voxel_x_, 1/voxel_param.voxel_y_, 1/voxel_param.voxel_z_;

        Eigen::Vector3f min_bbox, max_bbox, div_b, divb_mul;
        min_bbox << static_cast<int> (floor(min_p[0] * inverse_leaf_size[0])), static_cast<int> (floor(min_p[1] * inverse_leaf_size[1])), static_cast<int> (floor(min_p[2] * inverse_leaf_size[2]));
        max_bbox << static_cast<int> (floor(max_p[0] * inverse_leaf_size[0])), static_cast<int> (floor(max_p[1] * inverse_leaf_size[1])), static_cast<int> (floor(max_p[2] * inverse_leaf_size[2]));

        div_b << (max_bbox[0] - min_bbox[0] + 1), (max_bbox[1] - min_bbox[1] + 1), (max_bbox[2] - min_bbox[2] + 1);
        divb_mul << 1, div_b[0], div_b[0] * div_b[1];

        std::vector<PointCloudIndexIdx> index_vector;
        index_vector.reserve(input_point_cloud.size());

        //第一步：遍历所有点并将它们插入到具有计算idx的index_vector向量中;具有相同idx值的点将有助于产生CloudPoint的相同点
        for (int i = 0; i < input_point_cloud.size();i++)
        {
            int ijk0 = static_cast<int> (floor(input_point_cloud[i].GetX() * inverse_leaf_size[0]) - static_cast<float> (min_bbox[0]));
            int ijk1 = static_cast<int> (floor(input_point_cloud[i].GetY() * inverse_leaf_size[1]) - static_cast<float> (min_bbox[1]));
            int ijk2 = static_cast<int> (floor(input_point_cloud[i].GetZ() * inverse_leaf_size[2]) - static_cast<float> (min_bbox[2]));

            //计算质心叶索引
            int idx = ijk0 * divb_mul[0] + ijk1 * divb_mul[1] + ijk2 * divb_mul[2];
            index_vector.push_back(PointCloudIndexIdx(static_cast<unsigned int> (idx), i));
        }
        //第二步：使用表示目标单元格的值作为索引对index_vector向量进行排序;实际上属于同一输出单元格的所有点都将彼此相邻
        std::sort(index_vector.begin(), index_vector.end(), std::less<PointCloudIndexIdx>());

        //第三步：计数输出单元格，我们需要跳过所有相同的，相邻的idx值
        unsigned int total = 0;
        unsigned int index = 0;
        unsigned int min_points_per_voxel = 0;
        //first_and_last_indices_vector [i]表示属于对应于第i个输出点的体素的index_vector中的第一个点的index_vector中的索引，以及不属于第一个点的索引
        std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
        first_and_last_indices_vector.reserve(index_vector.size());                              //分配内存空间

        while (index < index_vector.size())
        {
            unsigned int i = index + 1;
            while (i < index_vector.size() && index_vector[i].idx_ == index_vector[index].idx_)
                ++i;
            if (i - index >= min_points_per_voxel)
            {
                ++total;
                first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int>(index, i));
            }
            index = i;
        }

        //第四步：计算质心，将它们插入最终位置
        //OutPointCloud.resize(total);      //给输出点云分配内存空间
        float x_Sum, y_Sum, z_Sum;
        PointXYZI<float> point;
        unsigned int first_index, last_index;
        for (unsigned int cp = 0; cp < first_and_last_indices_vector.size(); ++cp)
        {
            // 计算质心 - 来自所有输入点的和值，这些值在index_vector数组中具有相同的idx值
            first_index = first_and_last_indices_vector[cp].first;
            last_index = first_and_last_indices_vector[cp].second;
            x_Sum = 0;
            y_Sum = 0;
            z_Sum = 0;
            for (unsigned int li = first_index; li < last_index; ++li)
            {
                x_Sum += input_point_cloud[index_vector[li].point_cloud_index_].GetX();
                y_Sum += input_point_cloud[index_vector[li].point_cloud_index_].GetY();
                z_Sum += input_point_cloud[index_vector[li].point_cloud_index_].GetZ();
            }
            point.SetX(x_Sum / (last_index - first_index));
            point.SetY(y_Sum / (last_index - first_index));
            point.SetZ(z_Sum / (last_index - first_index));
            out_point_cloud.push_back(point);
        }

        return;
    }

    void VoxelFilter::GetMaxMin(PointCloud& input_point_cloud, Eigen::Vector3f& min_p, Eigen::Vector3f& max_p){
        if (input_point_cloud.size() == 0){
            printf("[%s]: Input Point Cloud is empty.\n", __func__);
            return;
        }

        float min_x = (*std::min_element(input_point_cloud.begin(), input_point_cloud.end(), [](PointXYZI<float>& a, PointXYZI<float>& b){return a.GetX() < b.GetX();})).GetX();
        float min_y = (*std::min_element(input_point_cloud.begin(), input_point_cloud.end(), [](PointXYZI<float>& a, PointXYZI<float>& b){return a.GetY() < b.GetY();})).GetY();
        float min_z = (*std::min_element(input_point_cloud.begin(), input_point_cloud.end(), [](PointXYZI<float>& a, PointXYZI<float>& b){return a.GetZ() < b.GetZ();})).GetZ();
        min_p << min_x, min_y, min_z;

        float max_x = (*std::max_element(input_point_cloud.begin(), input_point_cloud.end(), [](PointXYZI<float>& a, PointXYZI<float>& b){return a.GetX() < b.GetX(); })).GetX();
        float max_y = (*std::max_element(input_point_cloud.begin(), input_point_cloud.end(), [](PointXYZI<float>& a, PointXYZI<float>& b){return a.GetY() < b.GetY(); })).GetY();
        float max_z = (*std::max_element(input_point_cloud.begin(), input_point_cloud.end(), [](PointXYZI<float>& a, PointXYZI<float>& b){return a.GetZ() < b.GetZ(); })).GetZ();
        max_p << max_x, max_y, max_z;

        return;
    }

    void VoxelFilter::PclVoxel(PointCloud& input_point_cloud, PointCloud& out_point_cloud, VoxelParam voxel_param)
    {
        pcl::VoxelGrid<pcl::PointXYZI> filter;

        pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> out_cloud;
        for (size_t i = 0; i < input_point_cloud.size(); i++)
        {
            pcl::PointXYZI pcl_pt;
            pcl_pt.x = input_point_cloud[i].GetX();
            pcl_pt.y = input_point_cloud[i].GetY();
            pcl_pt.z = input_point_cloud[i].GetZ();
            pcl_pt.intensity = input_point_cloud[i].GetI();
            in_cloud->points.push_back(pcl_pt);
        }

        filter.setInputCloud(in_cloud);
        filter.setLeafSize(voxel_param.voxel_x_, voxel_param.voxel_y_, voxel_param.voxel_z_);
        filter.filter(out_cloud);

        for (int j = 0; j < out_cloud.points.size(); ++j) {
            PointXYZI<float> pt;
            pt.SetX(out_cloud.points[j].x);
            pt.SetY(out_cloud.points[j].y);
            pt.SetZ(out_cloud.points[j].z);
            pt.SetI(out_cloud.points[j].intensity);
            out_point_cloud.push_back(pt);
        }
    }

}

