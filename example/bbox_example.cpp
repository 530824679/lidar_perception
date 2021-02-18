#include "datas_load.h"
#include "render/render.h"
#include "process/lidar_process.h"

#include <ros/ros.h>
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

Render render;
ConfigManager config_manager;
std::shared_ptr<ROIFilter> roi_filter;
std::shared_ptr<VoxelFilter> voxel_filter;
std::shared_ptr<CurbDetect> curb_detect;
std::shared_ptr<Segment> segment;
std::shared_ptr<Cluster> object_cluster;
std::shared_ptr<LShapeBBoxEstimator> bbox_fitting;

void PreProcess(const PointCloudPtr& input_cloud_ptr, const hd_map_msgs::RouteData::ConstPtr p_route_ptr,
                PointCloudPtr& curb_cloud_ptr, std::vector<PointCloud>& out_cloud, ConfigManager config_manager)
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
    PointCloudPtr passthough_cloud_ptr = std::make_shared<PointCloud>();
    PointCloudPtr voxel_cloud_ptr = std::make_shared<PointCloud>();
    // PointCloudPtr curb_cloud_ptr = std::make_shared<PointCloud>();
    PointCloudPtr segment_cloud_ptr = std::make_shared<PointCloud>();

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

    roi_filter->PassThough(calibrate_cloud_ptr, p_route_ptr, passthough_cloud_ptr);
    voxel_filter->VoxelProcess(*passthough_cloud_ptr, *voxel_cloud_ptr);

    Eigen::Vector4d plane_coefficients;
    curb_detect->Detect(voxel_cloud_ptr, p_route_ptr, curb_cloud_ptr, plane_coefficients);
    segment->BuildGridMap(*curb_cloud_ptr, *segment_cloud_ptr, plane_coefficients);
    object_cluster->DBSCANClusterWithMerge(*segment_cloud_ptr, out_cloud);
}

void VisualizationResult(PointCloudPtr& curb_cloud_ptr, const std::vector<PointCloud>& cluster_cloud, std::vector<BBox>& bboxes)
{
    render.RenderDoubleBBox(curb_cloud_ptr, cluster_cloud, bboxes, "cluster", Color(0,1,0));
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
//     std::vector<PointCloud> filter_cloud;
//     PreProcess(out_cloud_ptr, filter_cloud, config_manager);

//     // Use to test moudle of voxel
//     clock_t start, end;
//     start = clock();

//     std::vector<BBox> bboxes;
//     bbox_fitting->Estimate(filter_cloud, bboxes);

//     end = clock();
//     cout << "The estimate bbox runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

//     VisualizationResult(filter_cloud, bboxes);
//     render.KeepWait();
// }

void RosbagData(const sensor_msgs::PointCloud2ConstPtr& p_Horizon_ptr, const hd_map_msgs::RouteData::ConstPtr p_route_ptr)
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

    // Use to test moudle of voxel
    clock_t start, end;
    start = clock();

    // Use to process the prior steps of passthough
    PointCloudPtr curb_cloud_ptr = std::make_shared<PointCloud>();
    std::vector<PointCloud> filter_cloud;
    PreProcess(out_cloud_ptr, p_route_ptr, curb_cloud_ptr, filter_cloud, config_manager);

    std::vector<BBox> bboxes;
    bbox_fitting->Estimate(filter_cloud, bboxes);

    end = clock();
    cout << "The estimate bbox runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

    VisualizationResult(curb_cloud_ptr, filter_cloud, bboxes);
    render.KeepOnce();
}

int main(int argc, char **argv){
    // set config
    string config_path = ros::package::getPath("obstacle_detection") + "/config/obstacle_detection.json";
    config_manager.SetConfig(config_path);
    roi_filter = std::make_shared<ROIFilter>(config_manager.GetROIParam());
    voxel_filter = std::make_shared<VoxelFilter>(config_manager.GetVoxelParam());
    curb_detect = std::make_shared<CurbDetect>(config_manager.GetROIParam(), config_manager.GetCurbParam());
    segment = std::make_shared<Segment>(config_manager.GetSegmentParam(), config_manager.GetROIParam());
    object_cluster = std::make_shared<Cluster>(config_manager.GetClusterParam());
    bbox_fitting = std::make_shared<LShapeBBoxEstimator>(config_manager.GetBBoxParam());

    /**********************
    Get datas from pcd
    **********************/
    // string pcd_path = "/home/chenwei/Project/perception_arm/635.pcd";
    // DATASLOAD datas_load(pcd_path);
    // datas_load.LoadPcd(PcdData);

    /**********************
    Get datas from rosbag
    **********************/
    ros::init(argc, argv, "bbox_example_ros");
    ros::NodeHandle node;
    // ros::Subscriber lidar_subscriber;
    // DATASLOAD datas_load("");
    // datas_load.LoadRosbag(node, lidar_subscriber, RosbagData);

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(node, "/livox/lidar", 1);
    message_filters::Subscriber<hd_map_msgs::RouteData> route_data(node, "/VMTP_RouteData", 1);

    typedef  message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, hd_map_msgs::RouteData> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lidar_sub, route_data);
    sync.registerCallback(boost::bind(&RosbagData, _1, _2));

    ros::spin();

    return 0;
}