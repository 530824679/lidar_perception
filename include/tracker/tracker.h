//
/******************************************************************************/
/*!
File name: tracker.h

Description:
This file define class of Tracker use to process of implementing the track.

Version: 0.1
Create date: 2020.5.11
Author: Chen Wei
Email: wei.chen@imotion.ai

Copyright (c) iMotion Automotive Technology (Suzhou) Co. Ltd. All rights reserved,
also regarding any disposal, exploitation, reproduction, editing, distribution,
as well as in the event of applications for industrial property rights.
*/
/******************************************************************************/

#ifndef _LIDAR_PERCEPTION_ROS_TRACKER_H_
#define _LIDAR_PERCEPTION_ROS_TRACKER_H_

#include "common/json/json.h"

// local include
#include "velocity.h"
#include "kalman_filter.h"

#include "common/utils/types.h"

namespace lidar_perception_ros{

    class Tracker {
    public:
        Tracker();
        ~Tracker();

        void Init(const BBox& bbox);
        void Predict();
        void Update(const BBox& bbox);
        TrackerInfo GetStateAsBbox() const;
        TrackerInfo GetStateAsInputBbox() const;
        float GetNIS() const;

        int GetCoastCycles();
        int GetHitStreak();

        bool RejectOutlier(TrackerInfo& bbox_);

    private:
        Point2D NearestBboxCorner(const BBox& bbox) const;
        Eigen::VectorXd ConvertBboxToObservation(const BBox& bbox) const;
        Eigen::VectorXd ConvertBottomToObservation(const BBox& bbox) const;
        TrackerInfo ConvertStateToBbox(const Eigen::VectorXd &state,const Eigen::VectorXd &velocity_state) const;
        TrackerInfo ConvertStateToInputBbox(const TrackerInfo &bbox) const;

    private:
        KalmanFilter kf_;
        KalmanFilter kf_velocity_;
        TrackerInfo bbox_;

        int coast_cycles_;
        int hit_streak_;

        int lidar_rate_;
        float acc_threshold_;

    };
}

#endif //_LIDAR_PERCEPTION_ROS_TRACKER_H_
