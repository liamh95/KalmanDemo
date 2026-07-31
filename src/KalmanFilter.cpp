
#include <eigen3/Eigen/Dense>
#include "KalmanFilter.h"


KalmanFilter::KalmanFilter(const Eigen::Vector4d& initial_state, const Eigen::Matrix4d& initial_covariance, double sigma_a)
    : x_ {initial_state}, P_ {initial_covariance}, sigma_a_ {sigma_a}{

}


void KalmanFilter::predict(double dt, const Eigen::Vector2d& u){
    // State model transition matrix F_k
    // 1    0   Delta t     0
    // 0    1   0           Delta t
    // 0    0   1           0
    // 0    0   0           1
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // Currently not used since we're using the default u = 0 for now.
    // Once IMU is part of control, this will be used
    Eigen::Matrix<double, 4, 2> B {{0.5 * dt * dt, 0}, {0, 0.5 * dt * dt}, {dt, 0}, {0, dt}};

    // Process noise covariance matrix Q_k
    // (Delta t)^4 / 4      0                   (Delta t)^3 / 2     0
    // 0                    (Delta t)^4 / 4     0                   (Delta t)^3 / 2
    // (Delta t)^3 / 2      0                   (Delta t)^2         0
    // 0                    (Delta t)^3 / 2     0                   (Delta t)^2
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = dt * dt * dt * dt / 4;
    Q(0, 2) = dt * dt * dt / 2;

    Q(1, 1) = dt * dt * dt * dt / 4;
    Q(1, 3) = dt * dt * dt / 2;

    Q(2, 0) = dt * dt * dt / 2;
    Q(2, 2) = dt * dt;
    
    Q(3, 1) = dt * dt * dt / 2;
    Q(3, 3) = dt * dt;
    
    // Q is a covariance, so it scales with the variance sigma_a^2, not sigma_a.
    Q *= sigma_a_ * sigma_a_;

    // prediction and propagation of P
    x_ = F * x_ + B * u;
    P_ = F * P_ * F.transpose() + Q;
}


void KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R){
    Eigen::MatrixXd kalman_matrix = P_ * H.transpose() * (R + H * P_ * H.transpose()).inverse();
    x_ = x_ + kalman_matrix * (z - H * x_);

    int state_space_dim = x_.size();

    // I guess (1-KH)P(1-KH)^T + KRK^T is "more numerically stable and robust" than (1 - KH)P
    // If that's the case, should I not precompute "factor" here?
    Eigen::MatrixXd factor = (Eigen::MatrixXd::Identity(state_space_dim, state_space_dim) - kalman_matrix * H);
    P_ =  factor * P_ * factor.transpose() + kalman_matrix * R * kalman_matrix.transpose();
}


bool KalmanFilter::gateAccepts(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R, const double threshold){
    Eigen::VectorXd diff = z - H * x_;
    Eigen::MatrixXd diff_cov = H * P_ * H.transpose() + R;

    return diff.transpose() * diff_cov.inverse() * diff < threshold;
}


Eigen::Vector4d KalmanFilter::state() const{
    return x_;
}

Eigen::Vector2d KalmanFilter::position() const{
    return x_.head<2>();
}


Eigen::Vector2d KalmanFilter::velocity() const{
    return x_.tail<2>();
}