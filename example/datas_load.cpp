#include "datas_load.h"

namespace lidar_perception_ros {

    DATASLOAD::DATASLOAD(string pcd_path)
    {
        pcd_path_ = pcd_path;
    }

    DATASLOAD::~DATASLOAD()
    {

    }

    void DATASLOAD::LoadRosbag(ros::NodeHandle& node, ros::Subscriber& subscriber, void(*RosbagData)(const sensor_msgs::PointCloud2ConstPtr& p_Horizon_ptr))
    {
        subscriber = node.subscribe<sensor_msgs::PointCloud2>("/livox/lidar_3GGDHAD00102871", 100, RosbagData);
    }

    void DATASLOAD::LoadPcd(void(*PcdData)(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr))
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path_, *in_cloud_ptr) == -1) {
            PCL_ERROR("PCD file reading failed.");
            return;
        }

        PcdData(in_cloud_ptr);
    }
}