
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int i=0;

void callback_save_pcd(const sensor_msgs::PointCloud2 &input) {

    //Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(input, cloud);

    //save to PCD
    std::string pcd_dir("/home/chenwei/HDD/livox_dl/wulin/pcd");
    std::string pcd_name = pcd_dir + std::to_string(i);
    i++;

    if (pcl::io::savePCDFileASCII(pcd_name + ".pcd", cloud) >= 0) {
        std::cout << "Saved  " << pcd_name << ".pcd" << std::endl;
    }
}

int main(int argc, char **argv) {

    // Initialize ROS
    ros::init(argc,argv,"save_pcd_example");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 100, callback_save_pcd);

    // Spin
    ros::spin();

    return 0;
}