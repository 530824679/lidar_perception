#include "render/render.h"

namespace lidar_perception_ros {

    Render::Render():v1(0), v2(0), viewer_(new pcl::visualization::PCLVisualizer ("3D Viewer"))
    {
        InitCamera();
        viewer_->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer_->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    }

    Render::~Render() {

    }

    void Render::InitCamera()
    {
        viewer_->setBackgroundColor(0, 0, 0);
        viewer_->addCoordinateSystem(1.0);
        viewer_->initCameraParameters();
        viewer_->setCameraPosition(-0, 0, 30, 1, 0, 1);
    }

    void Render::RenderGround()
    {
        // units in meters
        double roadLength = 40.0;
        double roadWidth = 40.0;
        double roadHeight = 0.1;

        viewer_->addCube(0, roadLength, -roadWidth/2, roadWidth/2, -roadHeight, 0, .2, .2, .2, "Ground");
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "Ground");
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, .2, .2, .2, "Ground");
        viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "Ground");
    }

    void Render::RenderPointCloud(const PointCloudPtr &input_cloud_ptr, std::string name, Color color)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        //RenderGround();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (size_t i = 0; i < (*input_cloud_ptr).size(); i++) {
            pcl::PointXYZ pt;
            pt.x = (*input_cloud_ptr)[i].GetX();
            pt.y = (*input_cloud_ptr)[i].GetY();
            pt.z = (*input_cloud_ptr)[i].GetZ();
            cloud.push_back(pt);
        }
        viewer_->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name);
    }

    void Render::RenderDoublePointCloud(const PointCloudPtr &before_cloud_ptr, const PointCloudPtr &after_cloud_ptr, std::string name, Color color)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        //RenderGround();
        pcl::PointCloud<pcl::PointXYZ> before_cloud;
        for (size_t i = 0; i < (*before_cloud_ptr).size(); i++) {
            pcl::PointXYZ pt;
            pt.x = (*before_cloud_ptr)[i].GetX();
            pt.y = (*before_cloud_ptr)[i].GetY();
            pt.z = (*before_cloud_ptr)[i].GetZ();
            before_cloud.push_back(pt);
        }

        pcl::PointCloud<pcl::PointXYZ> after_cloud;
        for (size_t i = 0; i < (*after_cloud_ptr).size(); i++) {
            pcl::PointXYZ pt;
            pt.x = (*after_cloud_ptr)[i].GetX();
            pt.y = (*after_cloud_ptr)[i].GetY();
            pt.z = (*after_cloud_ptr)[i].GetZ();
            after_cloud.push_back(pt);
        }


        viewer_->addPointCloud<pcl::PointXYZ>(before_cloud.makeShared(), name+"v1", v1);
        viewer_->addPointCloud<pcl::PointXYZ>(after_cloud.makeShared(), name+"v2", v2);

        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name+"v1", v1);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name+"v1", v1);

        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name+"v2", v2);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name+"v2", v2);
    }

    void Render::RenderCluster(const std::vector<PointCloud> &input_cloud, std::string name)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        //RenderGround();
        srand(time(NULL));
        for(int index = 0; index < input_cloud.size(); index++)
        {
            float random_r = rand() % 1000 / (float)1000;
            float random_g = rand() % 1000 / (float)1000;
            float random_b = rand() % 1000 / (float)1000;

            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (size_t i = 0; i < input_cloud[index].size(); i++) {
                pcl::PointXYZ pt;
                pt.x = input_cloud[index][i].GetX();
                pt.y = input_cloud[index][i].GetY();
                pt.z = input_cloud[index][i].GetZ();
                cloud.push_back(pt);
            }
            viewer_->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), name+to_string(index));
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name+to_string(index));
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, random_r, random_g, random_b, name+to_string(index));

        }
   }

    void Render::RenderDoubleCluster(const PointCloudPtr &input_cloud_ptr, const std::vector<PointCloud> &input_cloud, std::string name, Color color)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        //RenderGround();
        pcl::PointCloud<pcl::PointXYZ> object_cloud;
        for (size_t i = 0; i < (*input_cloud_ptr).size(); i++) {
            pcl::PointXYZ pt;
            pt.x = (*input_cloud_ptr)[i].GetX();
            pt.y = (*input_cloud_ptr)[i].GetY();
            pt.z = (*input_cloud_ptr)[i].GetZ();
            object_cloud.push_back(pt);
        }

        viewer_->addPointCloud<pcl::PointXYZ>(object_cloud.makeShared(), name+"v1", v1);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name+"v1", v1);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name+"v1", v1);

        srand(time(NULL));
        for(int index = 0; index < input_cloud.size(); index++)
        {
            float random_r = rand() % 1000 / (float)1000;
            float random_g = rand() % 1000 / (float)1000;
            float random_b = rand() % 1000 / (float)1000;

            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (size_t i = 0; i < input_cloud[index].size(); i++) {
                pcl::PointXYZ pt;
                pt.x = input_cloud[index][i].GetX();
                pt.y = input_cloud[index][i].GetY();
                pt.z = input_cloud[index][i].GetZ();
                cloud.push_back(pt);
            }
            viewer_->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), name+"v2"+to_string(index), v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name+"v2"+to_string(index), v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, random_r, random_g, random_b, name+"v2"+to_string(index), v2);

        }

    }

    void Render::RenderBBox(const std::vector<BBox> bboxes, Color color)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        //RenderGround();
        for (size_t i = 0; i < bboxes.size(); i++) {
            std::string cube = "box" + std::to_string(i);

            Eigen::Vector3f bboxTransform(bboxes[i].x, bboxes[i].y, bboxes[i].z);
            Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();

            matrix << cos(bboxes[i].rotate), -sin(bboxes[i].rotate), 0,
                    sin(bboxes[i].rotate), cos(bboxes[i].rotate), 0,
                    0, 0, 1;
            Eigen::Quaternionf bboxQuaternion(matrix);

            viewer_->addCube(bboxTransform, bboxQuaternion, bboxes[i].dx, bboxes[i].dy, bboxes[i].dz, cube);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0, 0, cube);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube);

        }
    }

    void Render::RenderDoubleBBox(const PointCloudPtr& curb_cloud_ptr, const std::vector<PointCloud>& cluster_cloud,
                                  const std::vector<BBox> bboxes, std::string name, Color color)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        //RenderGround();

        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (size_t i = 0; i < (*curb_cloud_ptr).size(); i++) {
            pcl::PointXYZ pt;
            pt.x = (*curb_cloud_ptr)[i].GetX();
            pt.y = (*curb_cloud_ptr)[i].GetY();
            pt.z = (*curb_cloud_ptr)[i].GetZ();
            cloud.push_back(pt);
        }
        viewer_->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), name+"v1", v1);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name+"v1", v1);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name+"v1", v1);

        srand(time(NULL));
        for(int index = 0; index < cluster_cloud.size(); index++)
        {
            float random_r = rand() % 1000 / (float)1000;
            float random_g = rand() % 1000 / (float)1000;
            float random_b = rand() % 1000 / (float)1000;

            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (size_t i = 0; i < cluster_cloud[index].size(); i++) {
                pcl::PointXYZ pt;
                pt.x = cluster_cloud[index][i].GetX();
                pt.y = cluster_cloud[index][i].GetY();
                pt.z = cluster_cloud[index][i].GetZ();
                cloud.push_back(pt);
            }
            viewer_->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), name+"v2"+to_string(index), v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name+"v2"+to_string(index), v2);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, random_r, random_g, random_b, name+"v2"+to_string(index), v2);

        }

        for (size_t i = 0; i < bboxes.size(); i++) {
            std::string cube = "box" + std::to_string(i);

            Eigen::Vector3f bboxTransform(bboxes[i].x, bboxes[i].y, bboxes[i].z);
            Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();

            matrix << cos(bboxes[i].rotate), -sin(bboxes[i].rotate), 0,
                    sin(bboxes[i].rotate), cos(bboxes[i].rotate), 0,
                    0, 0, 1;
            Eigen::Quaternionf bboxQuaternion(matrix);

            viewer_->addCube(bboxTransform, bboxQuaternion, bboxes[i].dx, bboxes[i].dy, bboxes[i].dz, cube, v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube, v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0, 0, cube, v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube, v2);
        }
    }

    void Render::RenderDLBBox(const PointCloudPtr &input_cloud_ptr, const std::vector<BBox> bboxes, std::string name, Color color)
    {
        viewer_->removeAllPointClouds();
        viewer_->removeAllShapes();
        //RenderGround();
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (size_t i = 0; i < (*input_cloud_ptr).size(); i++) {
            pcl::PointXYZ pt;
            pt.x = (*input_cloud_ptr)[i].GetX();
            pt.y = (*input_cloud_ptr)[i].GetY();
            pt.z = (*input_cloud_ptr)[i].GetZ();
            cloud.push_back(pt);
        }
        viewer_->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), name, v1);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name, v1);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name, v1);

        srand(time(NULL));
        for (size_t i = 0; i < bboxes.size(); i++) {
            std::string cube = "box" + std::to_string(i);

            Eigen::Vector3f bboxTransform(bboxes[i].x, bboxes[i].y, bboxes[i].z);
            Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();

            matrix << cos(bboxes[i].rotate), -sin(bboxes[i].rotate), 0,
                    sin(bboxes[i].rotate), cos(bboxes[i].rotate), 0,
                    0, 0, 1;
            Eigen::Quaternionf bboxQuaternion(matrix);

            viewer_->addCube(bboxTransform, bboxQuaternion, bboxes[i].dx, bboxes[i].dy, bboxes[i].dz, cube, v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube, v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0f, 0, 0, cube, v2);
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, cube, v2);
        }
        viewer_->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), name+"v2", v2);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.GetR(), color.GetG(), color.GetB(), name+"v2", v2);
    }

    void Render::KeepWait()
    {
        while (!viewer_->wasStopped ())
        {
            viewer_->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }

    void Render::KeepOnce()
    {
        viewer_->spinOnce(1);
    }
}