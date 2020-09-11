#include "manager/config_manager.h"

namespace lidar_perception_ros {

    ConfigManager::ConfigManager() {
    }

    ConfigManager::~ConfigManager() {

    }

    void ConfigManager::SetConfig(const std::string &config_path) {
        Json::Value root;
        if (config_path == "") {
            printf("[%s]: Config path is empty.\n", __func__);
            return;
        } else {
            std::ifstream file(config_path, std::ios::binary);
            if (!file.is_open()) {
                printf("[%s]: Error opening file.\n", __func__);
                return;
            }

            if (NULL != reader_.parse(file, root)) {
                printf("[%s]: Parse succeed.\n", __func__);

                SetCalibrateParam(root["calibrate"]);
                SetROIParam(root["roi_filter"]);
                SetVoxelParam(root["voxel_filter"]);
                SetSegmentParam(root["object_segment"]);
                SetCurbParam(root["curb_detect"]);
                SetClusterParam(root["object_cluster"]);
                SetBBoxParam(root["bbox_builder"]);
                SetTrackerParam(root["tracking"]);

                file.close();
                return;
            } else {
                printf("[%s]: Parse error.\n", __func__);
                file.close();
                return;
            }
        }
    }

    void ConfigManager::SetCalibrateParam(Json::Value param)
    {
        if (param.isMember("roll")) {
            calibrate_param_.roll_ = param["roll"].asDouble();
        } else {
            printf("[%s]: Has no member is roll.\n", __func__);
            return;
        }

        if (param.isMember("pitch")) {
            calibrate_param_.pitch_ = param["pitch"].asDouble();
        }else{
            printf("[%s]: Has no member is pitch.\n", __func__);
            return;
        }

        if (param.isMember("yaw")) {
            calibrate_param_.yaw_ = param["yaw"].asDouble();
        }else{
            printf("[%s]: Has no member is yaw.\n", __func__);
            return;
        }

        if (param.isMember("tx")) {
            calibrate_param_.tx_ = param["tx"].asDouble();
        }else{
            printf("[%s]: Has no member is tx.\n", __func__);
            return;
        }

        if (param.isMember("ty")) {
            calibrate_param_.ty_ = param["ty"].asDouble();
        }else{
            printf("[%s]: Has no member is ty.\n", __func__);
            return;
        }

        if (param.isMember("tz")) {
            calibrate_param_.tz_ = param["tz"].asDouble();
        }else{
            printf("[%s]: Has no member is tz.\n", __func__);
            return;
        }
    }

    void ConfigManager::SetROIParam(Json::Value param)
    {
        if (param.isMember("roi_x_min")) {
            roi_param_.roi_x_min_ = param["roi_x_min"].asInt();
        }else{
            printf("[%s]: Has no member is roi_x_min.\n", __func__);
            return;
        }

        if (param.isMember("roi_x_max")) {
            roi_param_.roi_x_max_ = param["roi_x_max"].asInt();
        }else{
            printf("[%s]: Has no member is roi_x_max.\n", __func__);
        }

        if (param.isMember("roi_y_min")) {
            roi_param_.roi_y_min_ = param["roi_y_min"].asInt();
        }else{
            printf("[%s]: Has no member is roi_y_min.\n", __func__);
            return;
        }

        if (param.isMember("roi_y_max")) {
            roi_param_.roi_y_max_ = param["roi_y_max"].asInt();
        }else{
            printf("[%s]: Has no member is roi_y_max.\n", __func__);
        }

        if (param.isMember("roi_z_min")) {
            roi_param_.roi_z_min_ = param["roi_z_min"].asDouble();
        }else{
            printf("[%s]: Has no member is roi_z_min.\n", __func__);
            return;
        }

        if (param.isMember("roi_z_max")) {
            roi_param_.roi_z_max_ = param["roi_z_max"].asDouble();
        }else{
            printf("[%s]: Has no member is roi_z_max.\n", __func__);
        }
    }

    void ConfigManager::SetVoxelParam(Json::Value param)
    {
        if (param.isMember("voxel_x")) {
            voxel_param_.voxel_x_ = param["voxel_x"].asDouble();
        }else{
            printf("[%s]: Has no member is voxel_x.\n", __func__);
        }

        if (param.isMember("voxel_y")) {
            voxel_param_.voxel_y_ = param["voxel_y"].asDouble();
        }else{
            printf("[%s]: Has no member is voxel_y.\n", __func__);
        }

        if (param.isMember("voxel_z")) {
            voxel_param_.voxel_z_ = param["voxel_z"].asDouble();
        }else{
            printf("[%s]: Has no member is voxel_z.\n", __func__);
        }
    }

    void ConfigManager::SetSegmentParam(Json::Value param)
    {
        if (param.isMember("max_iterations")) {
            segment_param_.max_iterations_ = param["max_iterations"].asInt();
        }else{
            printf("[%s]: Has no member is max_iterations.\n", __func__);
        }

        if (param.isMember("distance_tolerate")) {
            segment_param_.distance_tolerate_ = param["distance_tolerate"].asDouble();
        }else{
            printf("[%s]: Has no member is distance_tolerate.\n", __func__);
        }

        if (param.isMember("row")) {
            segment_param_.row_ = param["row"].asInt();
        }else{
            printf("[%s]: Has no member is row.\n", __func__);
        }

        if (param.isMember("column")) {
            segment_param_.column_ = param["column"].asInt();
        }else{
            printf("[%s]: Has no member is column.\n", __func__);
        }

        if (param.isMember("grid_size")) {
            segment_param_.grid_size_ = param["grid_size"].asDouble();
        }else{
            printf("[%s]: Has no member is grid_size.\n", __func__);
        }

        if (param.isMember("height_threshold")) {
            segment_param_.height_threshold_ = param["height_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is height_threshold.\n", __func__);
        }
    }

    void ConfigManager::SetCurbParam(Json::Value param)
    {
        if (param.isMember("grid_column")) {
            curb_param_.grid_column_ = param["grid_column"].asInt();
        }else{
            printf("[%s]: Has no member is grid_column.\n", __func__);
        }

        if (param.isMember("grid_row")) {
            curb_param_.grid_row_ = param["grid_row"].asInt();
        }else{
            printf("[%s]: Has no member is grid_row.\n", __func__);
        }

        if (param.isMember("min_curb_points")) {
            curb_param_.min_curb_points_ = param["min_curb_points"].asInt();
        }else{
            printf("[%s]: Has no member is min_curb_points.\n", __func__);
        }

        if (param.isMember("max_iterations")) {
            curb_param_.max_iterations_ = param["max_iterations"].asInt();
        }else{
            printf("[%s]: Has no member is max_iterations.\n", __func__);
        }

        if (param.isMember("min_ground_plane_points")) {
            curb_param_.min_ground_plane_points_ = param["min_ground_plane_points"].asInt();
        }else{
            printf("[%s]: Has no member is min_ground_plane_points.\n", __func__);
        }

        if (param.isMember("points_threshold")) {
            curb_param_.points_threshold_ = param["points_threshold"].asInt();
        }else{
            printf("[%s]: Has no member is points_threshold.\n", __func__);
        }

        if (param.isMember("min_single_curb_points")) {
            curb_param_.min_single_curb_points_ = param["min_single_curb_points"].asInt();
        }else{
            printf("[%s]: Has no member is min_single_curb_points.\n", __func__);
        }

        if (param.isMember("min_grid_z")) {
            curb_param_.min_grid_z_ = param["min_grid_z"].asDouble();
        }else{
            printf("[%s]: Has no member is min_grid_z.\n", __func__);
        }

        if (param.isMember("max_grid_z")) {
            curb_param_.max_grid_z_ = param["max_grid_z"].asDouble();
        }else{
            printf("[%s]: Has no member is max_grid_z.\n", __func__);
        }

        if (param.isMember("min_ground_plane_z")) {
            curb_param_.min_ground_plane_z_ = param["min_ground_plane_z"].asDouble();
        }else{
            printf("[%s]: Has no member is min_ground_plane_z.\n", __func__);
        }

        if (param.isMember("max_ground_plane_z")) {
            curb_param_.max_ground_plane_z_ = param["max_ground_plane_z"].asDouble();
        }else{
            printf("[%s]: Has no member is max_ground_plane_z.\n", __func__);
        }

        if (param.isMember("min_ground_plane_i")) {
            curb_param_.min_ground_plane_i_ = param["min_ground_plane_i"].asDouble();
        }else{
            printf("[%s]: Has no member is min_ground_plane_i.\n", __func__);
        }

        if (param.isMember("max_ground_plane_i")) {
            curb_param_.max_ground_plane_i_ = param["max_ground_plane_i"].asDouble();
        }else{
            printf("[%s]: Has no member is max_ground_plane_i.\n", __func__);
        }

        if (param.isMember("height_max_upper_threshold")) {
            curb_param_.height_max_upper_threshold_ = param["height_max_upper_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is height_max_upper_threshold.\n", __func__);
        }

        if (param.isMember("height_max_lower_threshold")) {
            curb_param_.height_max_lower_threshold_ = param["height_max_lower_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is height_max_lower_threshold.\n", __func__);
        }

        if (param.isMember("height_diff_upper_threshold")) {
            curb_param_.height_diff_upper_threshold_ = param["height_diff_upper_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is height_diff_upper_threshold.\n", __func__);
        }

        if (param.isMember("height_diff_lower_threshold")) {
            curb_param_.height_diff_lower_threshold_ = param["height_diff_lower_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is height_diff_lower_threshold.\n", __func__);
        }

        if (param.isMember("plane_dist_threshold")) {
            curb_param_.plane_dist_threshold_ = param["plane_dist_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is plane_dist_threshold.\n", __func__);
        }

        if (param.isMember("line_dist_threshold")) {
            curb_param_.line_dist_threshold_ = param["line_dist_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is line_dist_threshold.\n", __func__);
        }

        if (param.isMember("angle_threshold")) {
            curb_param_.angle_threshold_ = param["angle_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is angle_threshold.\n", __func__);
        }

        if (param.isMember("not_tracking_max")) {
            curb_param_.not_tracking_max_ = param["not_tracking_max"].asDouble();
        }else{
            printf("[%s]: Has no member is not_tracking_max_.\n", __func__);
        }
    }

    void ConfigManager::SetClusterParam(Json::Value param)
    {
        if (param.isMember("min_cluster_size")) {
            cluster_param_.min_cluster_size_ = param["min_cluster_size"].asInt();
        }else{
            printf("[%s]: Has no member is min_cluster_size.\n", __func__);
        }

        if (param.isMember("max_cluster_size")) {
            cluster_param_.max_cluster_size_ = param["max_cluster_size"].asInt();
        }else{
            printf("[%s]: Has no member is max_cluster_size.\n", __func__);
        }

        if (param.isMember("cluster_distance")) {
            cluster_param_.cluster_distance_ = param["cluster_distance"].asDouble();
        }else{
            printf("[%s]: Has no member is cluster_distance.\n", __func__);
        }
    }

    void ConfigManager::SetBBoxParam(Json::Value param)
    {
        if (param.isMember("min_distance")) {
            bbox_param_.min_distance_ = param["min_distance"].asDouble();
        } else {
            printf("[%s]: Has no member is min_distance.\n", __func__);
        }

        if (param.isMember("max_distance")) {
            bbox_param_.max_distance_ = param["max_distance"].asDouble();
        } else {
            printf("[%s]: Has no member is max_distance.\n", __func__);
        }

        if (param.isMember("box_min_bottom")) {
            bbox_param_.box_min_bottom_ = param["box_min_bottom"].asDouble();
        } else {
            printf("[%s]: Has no member is box_min_bottom.\n", __func__);
        }

        if (param.isMember("box_min_top")) {
            bbox_param_.box_min_top_ = param["box_min_top"].asDouble();
        } else {
            printf("[%s]: Has no member is box_min_top.\n", __func__);
        }

        if (param.isMember("box_min_volume")) {
            bbox_param_.box_min_volume_ = param["box_min_volume"].asDouble();
        } else {
            printf("[%s]: Has no member is box_min_volume.\n", __func__);
        }

        if (param.isMember("box_max_volume")) {
            bbox_param_.box_max_volume_ = param["box_max_volume"].asDouble();
        } else {
            printf("[%s]: Has no member is box_max_volume.\n", __func__);
        }

        if (param.isMember("height_threshold")) {
            bbox_param_.height_threshold_ = param["height_threshold"].asDouble();
        } else {
            printf("[%s]: Has no member is height_threshold.\n", __func__);
        }
    }

    void ConfigManager::SetTrackerParam(Json::Value param)
    {
        if (param.isMember("max_coast_cycles")) {
            tracker_param_.max_coast_cycles_ = param["max_coast_cycles"].asInt();
        }else{
            printf("[%s]: Has no member is max_coast_cycles.\n", __func__);
        }

        if (param.isMember("min_hits")) {
            tracker_param_.min_hits_ = param["min_hits"].asInt();
        }else{
            printf("[%s]: Has no member is min_hits.\n", __func__);
        }

        if (param.isMember("min_confidence")) {
            tracker_param_.min_confidence_ = param["min_confidence"].asDouble();
        }else{
            printf("[%s]: Has no member is min_confidence.\n", __func__);
        }

        if (param.isMember("lidar_rate")) {
            tracker_param_.lidar_rate_ = param["lidar_rate"].asInt();
        }else{
            printf("[%s]: Has no member is lidar_rate.\n", __func__);
        }

        if (param.isMember("acceleration_threshold")) {
            tracker_param_.acceleration_threshold_ = param["acceleration_threshold"].asInt();
        }else{
            printf("[%s]: Has no member is acceleration_threshold.\n", __func__);
        }

        if (param.isMember("filter_threshold")) {
            tracker_param_.filter_threshold_ = param["filter_threshold"].asDouble();
        }else{
            printf("[%s]: Has no member is filter_threshold.\n", __func__);
        }
    }

}