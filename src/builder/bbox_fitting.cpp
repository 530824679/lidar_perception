
#include "builder/bbox_fitting.h"

namespace lidar_perception_ros{

    BBoxEstimator::BBoxEstimator(BBoxParam param): bbox_filter_(param)
    {
        min_distance_ = param.min_distance_;
        max_distance_ = param.max_distance_;
    }

    BBoxEstimator::~BBoxEstimator()
    {

    }

    void BBoxEstimator::Estimate(std::vector<PointCloud> &clusters, std::vector<BBox> &bboxes)
    {
        int i = 0;
        for (int i = 0; i < clusters.size(); i++) {
            //if(i > 0) break;
            auto cluster = clusters[i];
            BBox box{};

            PointCloudPtr cluster_ptr = PointCloudPtr(new PointCloud (cluster));
            if (AABBFitting(cluster_ptr, box)){
                bboxes.push_back(box);
            }else{
                printf("Search Based Fitting false.\n");
            }
        }

        bbox_filter_.ConditionFilter(bboxes);
    }

    bool BBoxEstimator::AABBFitting(PointCloudPtr &in_cloud_ptr, BBox &box)
    {
        PointXYZI<float> min_point, max_point, centroid_point;
        GetMinMax3D(in_cloud_ptr, min_point, max_point, centroid_point);

        box.x = (min_point.GetX() + max_point.GetX()) / 2.0;
        box.y = (min_point.GetY() + max_point.GetY()) / 2.0;
        box.z = (min_point.GetZ() + max_point.GetZ()) / 2.0;

        box.centroid_x = centroid_point.GetX();
        box.centroid_y = centroid_point.GetY();
        box.centroid_z = centroid_point.GetZ();

        constexpr float ep = 0.001;
        box.dx = std::max(max_point.GetX() - min_point.GetX(), ep);
        box.dy = std::max(max_point.GetY() - min_point.GetY(), ep);
        box.dz = std::max(max_point.GetZ() - min_point.GetZ(), ep);

        return true;
    }

    bool BBoxEstimator::HullFitting(PointCloudPtr &in_cloud_ptr, BBox &box)
    {
        PointXYZI<float> centroid(0.0, 0.0, 0.0, 1.0);
        for (size_t i = 0; i < (*in_cloud_ptr).size(); i++)
        {
            centroid.SetX(centroid.GetX() + (*in_cloud_ptr)[i].GetX());
            centroid.SetY(centroid.GetY() + (*in_cloud_ptr)[i].GetY());
            centroid.SetZ(centroid.GetZ() + (*in_cloud_ptr)[i].GetZ());
        }

        box.centroid_x = (centroid.GetX() / (*in_cloud_ptr).size());
        box.centroid_y = (centroid.GetY() / (*in_cloud_ptr).size());
        box.centroid_z = (centroid.GetZ() / (*in_cloud_ptr).size());

        PointCloud hull;
        convex_hull_.GrahamScan(in_cloud_ptr, hull);
        convex_hull_.MinAreaRect(hull, box);

        float min_z = FLT_MAX;
        float max_z = -FLT_MAX;

        for (int i = 0; i < (*in_cloud_ptr).size(); i++)
        {
            if((*in_cloud_ptr)[i].GetZ() < min_z)
            {
                min_z = (*in_cloud_ptr)[i].GetZ();
            }

            if((*in_cloud_ptr)[i].GetZ() > max_z)
            {
                max_z = (*in_cloud_ptr)[i].GetZ();
            }
        }

        box.z = (max_z + min_z) / 2.0;

        constexpr float ep = 0.001;
        box.dz = std::max(max_z -min_z, ep);

        return true;
    }

    void BBoxEstimator::GetMinMax3D(const PointCloudPtr &in_cloud_ptr, PointXYZI<float>& min_point, PointXYZI<float>& max_point, PointXYZI<float>& centroid_point)
    {
        Eigen::Array3f min_pt, max_pt;
        min_pt.setConstant(FLT_MAX);
        max_pt.setConstant(-FLT_MAX);

        PointXYZI<float> centroid(0.0, 0.0, 0.0, 1.0);

        for (size_t i = 0; i < (*in_cloud_ptr).size(); i++)
        {
            Eigen::Array3f pt;
            pt << (*in_cloud_ptr).at(i).GetX(), (*in_cloud_ptr).at(i).GetY(), (*in_cloud_ptr).at(i).GetZ();
            min_pt = min_pt.min (pt);
            max_pt = max_pt.max (pt);

            centroid.SetX(centroid.GetX() + (*in_cloud_ptr)[i].GetX());
            centroid.SetY(centroid.GetY() + (*in_cloud_ptr)[i].GetY());
            centroid.SetZ(centroid.GetZ() + (*in_cloud_ptr)[i].GetZ());
        }

        centroid_point.SetX(centroid.GetX() / (*in_cloud_ptr).size());
        centroid_point.SetY(centroid.GetY() / (*in_cloud_ptr).size());
        centroid_point.SetZ(centroid.GetZ() / (*in_cloud_ptr).size());

        min_point.SetX(min_pt[0]); min_point.SetY(min_pt[1]); min_point.SetZ(min_pt[2]);
        max_point.SetX(max_pt[0]); max_point.SetY(max_pt[1]); max_point.SetZ(max_pt[2]);
    }
}