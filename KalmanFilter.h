
#ifndef KALMANFILTER_H
#define KALMANFILTER_H


#include <eigen3/Eigen/Dense>

class KalmanFilter{
private:

    /// @brief State space is 4-dimensional: [x, y, v_x, v_y]
    Eigen::Vector4d x_;


    /// @brief State estimate covariance: E[(x - x_)(x - x_)^T]
    Eigen::Matrix4d P_;


    /// @brief Standard deviation of process noise (a random acceleration)
    double sigma_a_;


public:

    KalmanFilter(const Eigen::Vector4d& initial_state, const Eigen::Matrix4d& initial_covariance, double sigma_a);

    /// @brief Builds the state and control matrices, then computes x_ = F * x_ + B * u_ and propagates covariance
    /// @param dt time step
    /// @param u control input vector. Defaulted to the zero vector
    void predict(double dt, const Eigen::Vector2d& u = Eigen::Vector2d::Zero());


    /// @brief Applies the Kalman filter to the state, given the measurement and its covariance
    /// @param z Measurement
    /// @param H Measurement matrix
    /// @param R Measurement covariance
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);


    /// @brief Uses a chi-square test to determine whether or not the innovation is an outlier. If so, gate off the measurement.
    /// @param z Measurement
    /// @param H Measurement matrix
    /// @param R Measurement covariance
    /// @param threshold Chi square threshold (e.g. 9.21 for 99% confidence)
    /// @return 
    bool gateAccepts(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R, const double threshold);


    /// @brief Returns the state vector x_
    /// @return The state x_
    Eigen::Vector4d state() const;


    /// @brief Extracts the estimated 2D position from the state estimate
    /// @return 2D position vector
    Eigen::Vector2d position() const;


    /// @brief Extracts the estimated 2D velocity from the state estimate
    /// @return 2D velocity vector
    Eigen::Vector2d velocity() const;
};

#endif