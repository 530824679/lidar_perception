/******************************************************************************/
/*!
File name: main_node.cpp

Description:
This file runs the node's main function of lidar perception

Create date: 2020.4.16
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <exception>
#include <cstdlib>
#include "process/lidar_process.h"

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "lidar_perception_ros");

        ros::Time::init();

        ros::NodeHandle nh;

        string config_path = "/home/chenwei/HDD/Project/lidar_perception/config/obstacle_detection.json";

        LidarProcess lidar_process;

        lidar_process.Init(config_path);

        ROS_INFO("Start Lidar Perception ROS loop\n");

        ros::spin();
    }
    catch (std::exception& e)
    {
        std::cerr << __FILE__ << ":" << __LINE__ << ":" << std::endl << "EXCEPTION '" << e.what() << "'" << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << __FILE__ << ":" << __LINE__ << ":" << std::endl << "caught non-std EXCEPTION!" << std::endl;
        return 1;
    }

    return 0;
}
