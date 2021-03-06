#include "tracker/kalman_filter.h"

namespace lidar_perception_ros{

    KalmanFilter::KalmanFilter(unsigned int num_states, unsigned int num_obs):
            num_states_(num_states), num_obs_(num_obs) {
        x_ = Eigen::VectorXd::Zero(num_states);
        x_predict_ = Eigen::VectorXd::Zero(num_states);
        F_ = Eigen::MatrixXd::Zero(num_states, num_states);
        P_ = Eigen::MatrixXd::Zero(num_states, num_states);
        P_predict_ = Eigen::MatrixXd::Zero(num_states, num_states);
        Q_ = Eigen::MatrixXd::Zero(num_states, num_states);
        H_ = Eigen::MatrixXd::Zero(num_obs, num_states);
        R_ = Eigen::MatrixXd::Zero(num_obs, num_obs);

        log_likelihood_delta_ = 0.0;
        NIS_ = 0.0;
    }

    void KalmanFilter::Coast() {
        x_predict_ = F_ * x_;
        P_predict_ = F_ * P_ * F_.transpose() + Q_;
    }

    void KalmanFilter::Predict() {
        Coast();
        x_ = x_predict_;
        P_ = P_predict_;
    }

    Eigen::VectorXd KalmanFilter::PredictionToObservation(const Eigen::VectorXd &state) {
        return (H_*state);
    }

    void KalmanFilter::Update(const Eigen::VectorXd& z) {
        Eigen::VectorXd z_predict = PredictionToObservation(x_predict_);

        Eigen::VectorXd y = z - z_predict;

        Eigen::MatrixXd Ht = H_.transpose();

        Eigen::MatrixXd S = H_ * P_predict_ * Ht + R_;

        NIS_ = y.transpose() * S.inverse() * y;

        Eigen::MatrixXd K = P_predict_ * Ht * S.inverse();//gain of kalman

        x_ = x_predict_ + K * y;//update gain

        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_states_, num_states_);

        P_ = (I - K * H_) * P_predict_;//update pk
    }

    float KalmanFilter::CalculateLogLikelihood(const Eigen::VectorXd& y, const Eigen::MatrixXd& S) {
        float log_likelihood;

        auto& L = S.llt().matrixL();

        float log_determinant = 0;
        for (unsigned int i = 0; i < S.rows(); i++){
            log_determinant += log(L(i, i));
        }
        log_determinant *= 2;

        log_likelihood = -0.5 * (y.transpose() * S.inverse() * y + num_obs_ * log(2 * M_PI) + log_determinant);

        if (std::isnan(log_likelihood)) {
            log_likelihood = -1e50;
        }

        return log_likelihood;
    }
}

