#include <filter/voxel_filter.h>
#include "process/lidar_process.h"

namespace lidar_perception_ros{

    LidarProcess::LidarProcess() : viewer_(new pcl::visualization::PCLVisualizer("3D Viewer"))
    {
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(1.0);
        viewer_->initCameraParameters();
        viewer_->setCameraPosition(-0, 0, 30, 1, 0, 1);
    }

    LidarProcess::~LidarProcess()
    {

    }

    bool LidarProcess::Init(const std::string &config_path)
    {
        config_manager_.SetConfig(config_path);

        // set extrinct
        CalibrateParam param = config_manager_.GetCalibrateParam();
        extrinsics_ = Eigen::Matrix4d::Identity();
        extrinsics_(0, 0) = cos(param.yaw_) * cos(param.pitch_);
        extrinsics_(0, 1) = -sin(param.yaw_) * cos(param.roll_) + cos(param.yaw_) * sin(param.pitch_) * sin(param.roll_);
        extrinsics_(0, 2) = sin(param.yaw_) * sin(param.roll_) + cos(param.yaw_) * sin(param.pitch_) * cos(param.roll_);

        extrinsics_(1, 0) = sin(param.yaw_) * cos(param.pitch_);
        extrinsics_(1, 1) = cos(param.yaw_) * cos(param.roll_) + sin(param.yaw_) * sin(param.pitch_) * sin(param.roll_);
        extrinsics_(1, 2) = -cos(param.yaw_) * sin(param.roll_) + sin(param.yaw_) * sin(param.pitch_) * cos(param.roll_);

        extrinsics_(2, 0) = -sin(param.pitch_);
        extrinsics_(2, 1) = cos(param.pitch_) * sin(param.roll_);
        extrinsics_(2, 2) = cos(param.pitch_) * cos(param.roll_);

        extrinsics_(0, 3) = param.tx_;
        extrinsics_(1, 3) = param.ty_;
        extrinsics_(2, 3) = param.tz_;

        // init sub module
        roi_filter_ = std::make_shared<ROIFilter>();
        outlier_filter_ = std::make_shared<OutlierFilter>();
        voxel_filter_ = std::make_shared<VoxelFilter>();
        segment_ = std::make_shared<Segment>(config_manager_.GetSegmentParam());
        curb_detect_ = std::make_shared<CurbDetect>(config_manager_.GetROIParam(), config_manager_.GetCurbParam());
        bbox_fitting_ = std::make_shared<BBoxEstimator>(config_manager_.GetBBoxParam());
        tracking_ = std::make_shared<Tracking>(config_manager_.GetTrackerParam());
    }

    void LidarProcess::run(ros::NodeHandle node)
    {
        lidar_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 100, &LidarProcess::callback, this);
        lidar_publisher_ = node_.advertise<visualization_msgs::Marker>("/test", 100);
    }

    void LidarProcess::callback(const sensor_msgs::PointCloud2ConstPtr& p_Horizon_ptr)
    {
        PointCloudPtr input_cloud_ptr = std::make_shared<PointCloud>();
        ConversionData(p_Horizon_ptr, input_cloud_ptr);

        lidar_perception::ObjectInfoArray object_info_msg;
        ProcessPointCloud(input_cloud_ptr, object_info_msg);
    }

    void LidarProcess::ProcessLidarData(const sensor_msgs::PointCloud2::ConstPtr& p_Horizon_ptr, lidar_perception::ObjectInfoArray &object_info_msg)
    {
        std::cout << "Enter into LidarProcess ProcessLidarData." << std::endl;
        PointCloudPtr input_cloud_ptr = std::make_shared<PointCloud>();
        ConversionData(p_Horizon_ptr, input_cloud_ptr);
        ProcessPointCloud(input_cloud_ptr, object_info_msg);

        return;
    }

    void LidarProcess::ConversionData(const sensor_msgs::PointCloud2::ConstPtr& input, PointCloudPtr& input_cloud_ptr)
    {
        std::cout << "Enter into LidarProcess ConversionData." << std::endl;
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl::fromROSMsg(*input, pcl_cloud);
        //std::cout << "size is :" << pcl_cloud.points.size() << std::endl;

        for (int i = 0; i < pcl_cloud.points.size(); ++i) {
            PointXYZI<float> pt;
            pt.SetX(pcl_cloud.points[i].x);
            pt.SetY(pcl_cloud.points[i].y);
            pt.SetZ(pcl_cloud.points[i].z);
            pt.SetI(pcl_cloud.points[i].intensity);
            (*input_cloud_ptr).push_back(pt);
        }

        CalibratePointCloud(input_cloud_ptr);

    }

    void LidarProcess::CalibratePointCloud(PointCloudPtr& input_cloud_ptr)
    {
        for (size_t i = 0; i < (*input_cloud_ptr).size(); i++)
        {
            PointXYZI<float> pt = (*input_cloud_ptr)[i];
            (*input_cloud_ptr)[i].SetX(static_cast<float> (extrinsics_ (0, 0) * pt.GetX() + extrinsics_ (0, 1) * pt.GetY() + extrinsics_ (0, 2) * pt.GetZ() + extrinsics_ (0, 3)));
            (*input_cloud_ptr)[i].SetY(static_cast<float> (extrinsics_ (1, 0) * pt.GetX() + extrinsics_ (1, 1) * pt.GetY() + extrinsics_ (1, 2) * pt.GetZ() + extrinsics_ (1, 3)));
            (*input_cloud_ptr)[i].SetZ(static_cast<float> (extrinsics_ (2, 0) * pt.GetX() + extrinsics_ (2, 1) * pt.GetY() + extrinsics_ (2, 2) * pt.GetZ() + extrinsics_ (2, 3)));
            (*input_cloud_ptr)[i].SetI(pt.GetI());
        }
    }

    void LidarProcess::ProcessPointCloud(const PointCloudPtr& input_cloud_ptr, lidar_perception::ObjectInfoArray& object_info_msg)
    {
        clock_t start, end;
        start = clock();
        PointCloudPtr filter_cloud_ptr(new PointCloud);
        roi_filter_->PassThough(input_cloud_ptr, filter_cloud_ptr, config_manager_.GetROIParam());
        end = clock();
        cerr << "The filter roi runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
        cerr << "After the filter cloud num is: " << (*filter_cloud_ptr).size() << std::endl;

//        start = clock();
//        PointCloudPtr inlier_cloud_ptr(new PointCloud);
//        outlier_filter_->OutlierRemove(*filter_cloud_ptr, *inlier_cloud_ptr);
//        end = clock();
//        cerr << "The oulier remove runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

        start = clock();
        PointCloudPtr voxel_cloud_ptr(new PointCloud);
        voxel_filter_->PclVoxel(*filter_cloud_ptr, *voxel_cloud_ptr, config_manager_.GetVoxelParam());
        end = clock();
        cerr << "The voxel cloud runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
        cerr << "After the voxel cloud num is: " << (*voxel_cloud_ptr).size() << std::endl;

        start = clock();
        PointCloudPtr curb_filter_ptr(new PointCloud);
        curb_detect_->Detect(voxel_cloud_ptr, curb_filter_ptr);
        end = clock();
        cerr << "The curb filter runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
        cerr << "After the curb cloud num is: " << (*curb_filter_ptr).size() << std::endl;

        start = clock();
        PointCloudPtr object_cloud_ptr(new PointCloud);
        segment_->BuildGridMap(*voxel_cloud_ptr, *object_cloud_ptr);
        end = clock();
        cerr << "The object segment runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
        cerr << "After the segment cloud num is: " << (*object_cloud_ptr).size() << std::endl;

        start = clock();
        std::vector<PointCloud> cluster_cloud;
        object_cluster_->PclEuclidCluster(*object_cloud_ptr, cluster_cloud, config_manager_.GetClusterParam());
        end = clock();
        cerr << "The object cluster runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

        start = clock();
        std::vector<BBox> bboxes;
        bbox_fitting_->Estimate(cluster_cloud, bboxes, object_info_msg);
        end = clock();
        cerr << "The BBox runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;
       
        start = clock();
        tracking_->Process(bboxes, object_info_msg);
        end = clock();
        cerr << "The Tracking runtime is: " << (float)(end - start) * 1000 / CLOCKS_PER_SEC << "ms" << std::endl;

        // PCL Visualization
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();


        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (size_t i = 0; i < (*filter_cloud_ptr).size(); i++) {
            pcl::PointXYZ pt;
            pt.x = (*filter_cloud_ptr)[i].GetX();
            pt.y = (*filter_cloud_ptr)[i].GetY();
            pt.z = (*filter_cloud_ptr)[i].GetZ();
            cloud.push_back(pt);
        }
        viewer_->addPointCloud<pcl::PointXYZ> (cloud.makeShared(), "PointCloud");
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PointCloud");
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5f, 0.5f, 0.0f, "PointCloud");


        for(int k = 0; k < cluster_cloud.size(); k++)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (size_t i = 0; i < (cluster_cloud[k]).size(); i++) {
                pcl::PointXYZ pt;
                pt.x = (cluster_cloud[k])[i].GetX();
                pt.y = (cluster_cloud[k])[i].GetY();
                pt.z = (cluster_cloud[k])[i].GetZ();
                cloud.push_back(pt);
            }
            viewer_->addPointCloud<pcl::PointXYZ> (cloud.makeShared(), "PointCloud"+to_string(k));
            viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PointCloud"+to_string(k));
            viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0f, 0.5f, 0.5f, "PointCloud"+to_string(k));


        }


        int clusterid = 0;
        for (size_t i = 0; i < bboxes.size(); i++) {
            std::string cube = "box" + std::to_string(i);
            Eigen::Vector3f bboxTransform(bboxes[i].x, bboxes[i].y, bboxes[i].z);
            Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();
            matrix << cos(bboxes[i].yaw), sin(bboxes[i].yaw), 0,
                    -sin(bboxes[i].yaw), cos(bboxes[i].yaw), 0,
                    0, 0, 1;
            Eigen::Quaternionf bboxQuaternion(matrix);
            viewer_->addCube(bboxTransform, bboxQuaternion, bboxes[i].dx, bboxes[i].dy, bboxes[i].dz, cube);
            //viewer_->addCube(bboxes[i].x-bboxes[i].dx/2, bboxes[i].x+bboxes[i].dx/2, bboxes[i].y-bboxes[i].dy/2, bboxes[i].y+bboxes[i].dy/2, bboxes[i].z-bboxes[i].dz/2, bboxes[i].z+bboxes[i].dz/2, 1.0, 0, 0, cube);

            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0, 0, cube);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);

            clusterid++;
        }

//        int tracking_id = 0;
//        for (size_t i = 0; i < object_info_msg.tracking_object_num; i++) {
//            std::string cube = "tracking_box" + std::to_string(i);
//            std::string text = "text" + std::to_string(i);
//            std::string velocity_x = "velocity_x"+std::to_string(i);
//            std::string velocity_y = "velocity_y"+std::to_string(i);
//
//            pcl::PointXYZ startPoint;
//            startPoint.x=(float)object_info_msg.tracking_object_info[i].tk_distance_xv/128;
//            startPoint.y=(float)object_info_msg.tracking_object_info[i].tk_distance_yv/128;
//            startPoint.z=(float)object_info_msg.tracking_object_info[i].tk_center_z/128;
//
//            pcl::PointXYZ v_x_Point;
//            v_x_Point.x=(float)object_info_msg.tracking_object_info[i].tk_distance_xv/128+2;
//            v_x_Point.y=(float)object_info_msg.tracking_object_info[i].tk_distance_yv/128;
//            v_x_Point.z=(float)object_info_msg.tracking_object_info[i].tk_center_z/128;
//
//            pcl::PointXYZ v_y_Point;
//            v_y_Point.x=(float)object_info_msg.tracking_object_info[i].tk_distance_xv/128;
//            v_y_Point.y=(float)object_info_msg.tracking_object_info[i].tk_distance_yv/128+2;
//            v_y_Point.z=(float)object_info_msg.tracking_object_info[i].tk_center_z/128;
//
//        //    Eigen::Vector3f bboxTransform(bboxes[i].x, bboxes[i].y, bboxes[i].z);
//        //    Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();
//        //    matrix << cos(bboxes[i].yaw), sin(bboxes[i].yaw), 0,
//        //            -sin(bboxes[i].yaw), cos(bboxes[i].yaw), 0,
//        //            0, 0, 1;
//        //    Eigen::Quaternionf bboxQuaternion(matrix);
//        //    viewer_->addCube(bboxTransform, bboxQuaternion, bboxes[i].dx, bboxes[i].dy, bboxes[i].dz, cube);
//            viewer_->addCube((float)object_info_msg.tracking_object_info[i].tk_distance_xv/128-(float)object_info_msg.tracking_object_info[i].tk_length/256,
//            (float)object_info_msg.tracking_object_info[i].tk_distance_xv/128+(float)object_info_msg.tracking_object_info[i].tk_length/256,
//            (float)object_info_msg.tracking_object_info[i].tk_distance_yv/128-(float)object_info_msg.tracking_object_info[i].tk_width/256,
//            (float)object_info_msg.tracking_object_info[i].tk_distance_yv/128+(float)object_info_msg.tracking_object_info[i].tk_width/256,
//            (float)object_info_msg.tracking_object_info[i].tk_center_z/128-(float)object_info_msg.tracking_object_info[i].tk_height/256,
//            (float)object_info_msg.tracking_object_info[i].tk_center_z/128+(float)object_info_msg.tracking_object_info[i].tk_height/256, 0, 1, 0, cube);
//
//            std::string text_show="id:"+std::to_string(object_info_msg.tracking_object_info[i].id);
//
//            viewer_->addText3D(text_show,startPoint,0.8,123,22,44,text);
//            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
//            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1.0f, 0, cube);
//            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);
//
//
//            tracking_id++;
//        }

        viewer_->spinOnce();


//        visualization_msgs::Marker points;
//        points.header.frame_id = "livox_frame";
//        points.header.stamp = ros::Time::now();
//        points.action = visualization_msgs::Marker::MODIFY;
//        points.pose.orientation.w = 1.0;
//        points.type = visualization_msgs::Marker::POINTS;
//
//        points.scale.x = 0.2;
//        points.scale.y = 0.2;
//        points.scale.z = 0.2;
//
//        //srand(time(NULL));
//        points.color.r = 0;
//        points.color.g = 1.0f;
//        points.color.b = 0;//rand() % 10 / (float)(10);
//        points.color.a = 0.5f;
//
//        for (int i = 0; i < (*curb_filter_ptr).size(); ++i)
//        {
//            geometry_msgs::Point pt;
//            pt.x = (*curb_filter_ptr)[i].GetX();
//            pt.y = (*curb_filter_ptr)[i].GetY();
//            pt.z = (*curb_filter_ptr)[i].GetZ();
//
//            points.points.push_back(pt);
//        }
//
//        lidar_publisher_.publish(points);

    }
}
