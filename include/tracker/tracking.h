//
/******************************************************************************/
/*!
File name: tracking.h

Description:
This file define class of Tracking use to realize function of match and track.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_TRACKING_H_
#define _LIDAR_PERCEPTION_ROS_TRACKING_H_

// system include
//#include <iostream>
//#include <vector>
//#include <string.h>

// local include
#include "tracker.h"
#include "common/utils/point3d.h"
#include "munkres.h"
#include "tracker/rotated_iou.h"

//opencv include
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/opencv.hpp>

namespace lidar_perception_ros{

    class Tracking{
    public:
        Tracking() = default;
        Tracking(TrackerParam param);
        ~Tracking();

        void Process(std::vector<BBox> bboxes);
        float CalculateIou(const BBox& det, const Tracker& track);
        float CalculateRotateIOU(const BBox& det,const Tracker& track);
        float CalculateLocationDistance(const BBox& det, const Tracker& track);
        float CalculateCentroidDistance(const BBox& det, const Tracker& track);
        float CalculatePointnumDistance(const BBox& det, const Tracker& track);
        void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix, size_t nrows, size_t ncols, std::vector<std::vector<float>>& association);
        void AssociateDetectionsToTrackers(const std::vector<BBox> &bboxes,
                                           std::map<int, Tracker>& tracks,
                                           std::map<int, BBox>& matched,
                                           std::vector<BBox>& unmatched_det,
                                           float iou_threshold = 0.2);
        int Track(std::map<int, Tracker> &tracks, std::vector<BBox> bboxes, int frame_index, int& current_id);

    private:
        std::map<int, Tracker> tracks_;
        int frame_index_;
        int current_id_;
        Velocity filter_velocity_;
        Velocity publish_velocity_;
        rotated_iou rotated_iou_;

        int max_coast_cycles_;//允许的最大的失去检测的帧数
        int min_hits_;
        int lidar_rate_;
        int acceleration_threshold_;
        float min_confidence_;
        float filter_threshold_;
        int kernel_size_;
        float iou_weight_;
        float rotate_iou_weight_;
        float centroid_weight_;
        float distance_weight_;
        float pointnum_weight_;
        bool use_mutil_threshold_;
        float first_threshold_;
        float second_threshold_;
        float third_threshold_;
        int tracking_distance_[4];
    };
}

#endif //_LIDAR_PERCEPTION_ROS_TRACKING_H_
