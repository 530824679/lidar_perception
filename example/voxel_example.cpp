#include "datas_load.h"
#include "render/render.h"
#include "process/lidar_process.h"

#include <ros/ros.h>
#include <chrono>
#include <message_filters/subscriber.h>

Render render;
ConfigManager config_manager;
std::shared_ptr<ROIFilter> roi_filter;
std::shared_ptr<VoxelFilter> voxel_filter;

void PreProcess(const PointCloudPtr& input_cloud_ptr, PointCloudPtr out_cloud_ptr, ConfigManager config_manager)
{
    // set extrinct
    CalibrateParam param = config_manager.GetCalibrateParam();
    Eigen::Matrix4d extrinsics = Eigen::Matrix4d::Identity();
    extrinsics(0, 0) = cos(param.yaw_) * cos(param.pitch_);
    extrinsics(0, 1) = -sin(param.yaw_) * cos(param.roll_) + cos(param.yaw_) * sin(param.pitch_) * sin(param.roll_);
    extrinsics(0, 2) = sin(param.yaw_) * sin(param.roll_) + cos(param.yaw_) * sin(param.pitch_) * cos(param.roll_);

    extrinsics(1, 0) = sin(param.yaw_) * cos(param.pitch_);
    extrinsics(1, 1) = cos(param.yaw_) * cos(param.roll_) + sin(param.yaw_) * sin(param.pitch_) * sin(param.roll_);
    extrinsics(1, 2) = -cos(param.yaw_) * sin(param.roll_) + sin(param.yaw_) * sin(param.pitch_) * cos(param.roll_);

    extrinsics(2, 0) = -sin(param.pitch_);
    extrinsics(2, 1) = cos(param.pitch_) * sin(param.roll_);
    extrinsics(2, 2) = cos(param.pitch_) * cos(param.roll_);

    extrinsics(0, 3) = param.tx_;
    extrinsics(1, 3) = param.ty_;
    extrinsics(2, 3) = param.tz_;

    // calibrate point cloud
    PointCloudPtr calibrate_cloud_ptr = std::make_shared<PointCloud>();
    for (size_t i = 0; i < (*input_cloud_ptr).size(); i++)
    {
        PointXYZI<float> pt = (*input_cloud_ptr)[i];
        PointXYZI<float> out;
        out.SetX(static_cast<float> (extrinsics (0, 0) * pt.GetX() + extrinsics (0, 1) * pt.GetY() + extrinsics (0, 2) * pt.GetZ() + extrinsics (0, 3)));
        out.SetY(static_cast<float> (extrinsics (1, 0) * pt.GetX() + extrinsics (1, 1) * pt.GetY() + extrinsics (1, 2) * pt.GetZ() + extrinsics (1, 3)));
        out.SetZ(static_cast<float> (extrinsics (2, 0) * pt.GetX() + extrinsics (2, 1) * pt.GetY() + extrinsics (2, 2) * pt.GetZ() + extrinsics (2, 3)));
        out.SetI(pt.GetI());
        (*calibrate_cloud_ptr).push_back(out);
    }

    roi_filter->PassThough(calibrate_cloud_ptr, out_cloud_ptr);
}

void VisualizationResult(const PointCloudPtr& before_cloud_ptr, const PointCloudPtr& after_cloud_ptr)
{
    render.RenderDoublePointCloud(before_cloud_ptr, after_cloud_ptr, "PointCloud", Color(0,1,0));

}

// void PcdData(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr)
// {
//     PointCloudPtr out_cloud_ptr = std::make_shared<PointCloud>();
//     for (int i = 0; i < input_cloud_ptr->points.size(); ++i) {
//         PointXYZI<float> pt;
//         pt.SetX(input_cloud_ptr->points[i].x);
//         pt.SetY(input_cloud_ptr->points[i].y);
//         pt.SetZ(input_cloud_ptr->points[i].z);
//         pt.SetI(1.0);
//         (*out_cloud_ptr).push_back(pt);
//     }

//     // Use to process the prior steps of passthough
//     PointCloudPtr filter_cloud_ptr = std::make_shared<PointCloud>();
//     PreProcess(out_cloud_ptr, filter_cloud_ptr, config_manager);

//     // Use to test moudle of voxel
//     clock_t start, end;
//     start = clock();

//     PointCloudPtr result_cloud_ptr = std::make_shared<PointCloud>();
//     voxel_filter->VoxelProcess(*filter_cloud_ptr, *result_cloud_ptr);

//     end = clock();
//     cout << "The filter voxel runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
//     cout << "Before the voxel cloud num is: " << (*filter_cloud_ptr).size() << std::endl;
//     cout << "After the voxel cloud num is: " << (*result_cloud_ptr).size() << std::endl;

//     // Visual
//     VisualizationResult(filter_cloud_ptr, result_cloud_ptr);
//     render.KeepWait();
// }

void RosbagData(const sensor_msgs::PointCloud2ConstPtr& p_Horizon_ptr)
{
    sensor_msgs::PointCloud input_pointcloud;
    sensor_msgs::convertPointCloud2ToPointCloud((*p_Horizon_ptr), input_pointcloud);

    PointCloudPtr out_cloud_ptr = std::make_shared<PointCloud>();
    for (int i = 0; i < input_pointcloud.points.size(); ++i) {
        PointXYZI<float> pt;
        pt.SetX(input_pointcloud.points[i].x);
        pt.SetY(input_pointcloud.points[i].y);
        pt.SetZ(input_pointcloud.points[i].z);
        pt.SetI(input_pointcloud.channels[0].values[i]);
        (*out_cloud_ptr).push_back(pt);
    }

    // Use to process the prior steps of passthough
    PointCloudPtr filter_cloud_ptr = std::make_shared<PointCloud>();
    PreProcess(out_cloud_ptr, filter_cloud_ptr, config_manager);

    // Use to test moudle of voxel
    clock_t start, end;
    start = clock();

    PointCloudPtr result_cloud_ptr = std::make_shared<PointCloud>();
    voxel_filter->VoxelProcess(*filter_cloud_ptr, *result_cloud_ptr);

    end = clock();
    cout << "The filter voxel runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
    cout << "Before the voxel cloud num is: " << (*filter_cloud_ptr).size() << std::endl;
    cout << "After the voxel cloud num is: " << (*result_cloud_ptr).size() << std::endl;

    VisualizationResult(filter_cloud_ptr, result_cloud_ptr);
    render.KeepOnce();
}

int main(int argc, char **argv){
    // set config
    string config_path = "/home/chenwei/HDD/Project/lidar_perception/config/obstacle_detection.json";
    config_manager.SetConfig(config_path);
    roi_filter = std::make_shared<ROIFilter>(config_manager.GetROIParam());
    voxel_filter = std::make_shared<VoxelFilter>(config_manager.GetVoxelParam());

    /**********************
    Get datas from pcd
    **********************/
    // string pcd_path = "/home/chenwei/1.pcd";
    // DATASLOAD datas_load(pcd_path);
    // datas_load.LoadPcd(PcdData);

    /**********************
    Get datas from rosbag
    **********************/
    ros::init(argc, argv, "voxel_example_ros");
    ros::NodeHandle node;
    ros::Subscriber lidar_subscriber;
    DATASLOAD datas_load("");
    datas_load.LoadRosbag(node, lidar_subscriber, RosbagData);

    ros::spin();

    return 0;
}