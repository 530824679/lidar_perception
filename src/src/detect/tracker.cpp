#include "detect/tracker.h"

Tracker::Tracker() : kf_(4, 2)
{
    /*** Define constant velocity model ***/
    // state - k, b, v_k, v_b
    kf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // Give high uncertainty to the unobservable initial velocities
    kf_.P_ << 10, 0, 0, 0,
        0, 10, 0, 0,
        0, 0, 10000, 0,
        0, 0, 0, 10000;

    kf_.H_ << 1, 0, 0, 0,
        0, 1, 0, 0;

    kf_.Q_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0.0001, 0,
        0, 0, 0, 0.0001;

    kf_.R_ << 1, 0,
        0, 10;
}

Tracker::~Tracker() {}

// Get predicted locations from existing trackers
// dt is time elapsed between the current and previous measurements
void Tracker::Predict()
{
    kf_.Predict();

    // hit streak count will be reset
    if (coast_cycles_ > 0)
    {
        hit_streak_ = 0;
    }
    // accumulate coast cycle count
    coast_cycles_++;
}

// Update matched trackers with assigned detections
void Tracker::Update(const Eigen::Vector2d& kb)
{
    // get measurement update, reset coast cycle count
    coast_cycles_ = 0;
    // accumulate hit streak count
    hit_streak_++;

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = ConvertBboxToObservation(kb);

    kf_.Update(observation);
}

// Create and initialize new trackers for unmatched detections, with initial bounding box
void Tracker::Init(const Eigen::Vector2d& kb)
{
    kf_.x_.head(2) << ConvertBboxToObservation(kb);

    hit_streak_++;
}

/**
 * Returns the current bounding box estimate
 * @return
 */
Eigen::Vector2d Tracker::GetStateAsBbox() const
{
    return ConvertStateToBbox(kf_.x_);
}

float Tracker::GetNIS() const
{
    return kf_.NIS_;
}

/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [x, y, s, r] where x,y is the centre of the box and s is the scale/area and r is
 * the aspect ratio
 *
 * @param bbox
 * @return
 */
Eigen::VectorXd Tracker::ConvertBboxToObservation(const Eigen::Vector2d& kb) const
{
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(2);
    observation << kb(0, 0), kb(1, 0);
    return observation;
}

/**
 * Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
 * [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 *
 * @param state
 * @return
 */
Eigen::Vector2d Tracker::ConvertStateToBbox(const Eigen::VectorXd& state) const
{
    // state - k, b, v_k, v_b

    Eigen::Vector2d kb;
    kb(0, 0) = state[0];
    kb(1, 0) = state[1];

    return kb;
}
