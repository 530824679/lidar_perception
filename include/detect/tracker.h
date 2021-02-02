#pragma once

#include <math.h>

#include "kalman_filter.h"

class Tracker
{
public:
    // Constructor
    Tracker();
    ~Tracker();

    void Init(const Eigen::Vector2d& kb);
    void Predict();
    void Update(const Eigen::Vector2d& kb);
    Eigen::Vector2d GetStateAsBbox() const;

    float GetNIS() const;

    int coast_cycles_ = 0;
    int hit_streak_ = 0;

private:
    Eigen::VectorXd ConvertBboxToObservation(const Eigen::Vector2d& kb) const;
    Eigen::Vector2d ConvertStateToBbox(const Eigen::VectorXd& state) const;

    KalmanFilter kf_;
};
