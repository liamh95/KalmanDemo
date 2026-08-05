#ifndef MEASUREMENT_MODEL_H
#define MEASUREMENT_MODEL_H


#include <eigen3/Eigen/Core>
#include "Angles.h"

/// @brief Extended Kalman filter measurement model. Assumes measurement evolves via 
///        z_k = h(x_k) + v_k, 
///        where v_k is a centered multivariate Gaussian.
struct MeasurementModel{
    Eigen::MatrixXd R; // observation noise covariance


    /// @brief h(x), where the measurement evolves via z_k = h(x_k) + v_k.
    /// @param x State vector. 
    ///          [x, y, vx, vy] in constant-velocity model
    ///          [x, y, vx, vy, ax, ay] in constant-acceleration model
    /// @return h(x)
    virtual Eigen::VectorXd h(const Eigen::VectorXd& x) const = 0;


    /// @brief Jacobian of h(x), where the measurement evolves via x_k = h(x_k) + v_k.
    /// @param x State vector. 
    ///          [x, y, vx, vy] in constant-velocity model
    ///          [x, y, vx, vy, ax, ay] in constant-acceleration model
    /// @return Jacobian matrix H(x).
    virtual Eigen::MatrixXd H(const Eigen::VectorXd& x) const = 0;


    /// @brief AKA Innovation. Difference between measurement and estimate, where measurement
    ///        evolves via z_k = h(x_k) = v_k.
    /// @param z Measurement
    /// @param hx h applied to x_k.
    /// @return y_k = z_k - h(x_k).
    virtual Eigen::VectorXd residual(const Eigen::VectorXd& z, const Eigen::VectorXd& hx) const;

    virtual ~MeasurementModel() = default;

};

/// @brief Measurement model for a GPS sensor.
struct GPSModel: public MeasurementModel{
    
    /// @brief Returns H*x since this measurement is assumed to be linear.
    /// @param x State vector. 
    ///          [x, y, vx, vy] in constant-velocity model
    ///          [x, y, vx, vy, ax, ay] in constant-acceleration model
    /// @return H * x, where H is either
    ///         [1, 0, 0, 0], 
    ///         [0, 1, 0, 0]
    ///         in the constant-velocity model or 
    ///         [1, 0, 0, 0, 0, 0], 
    ///         [0, 1, 0, 0, 0, 0]
    ///         in the constant-acceleration model.
    Eigen::VectorXd h(const Eigen::VectorXd& x) const override;

    
    /// @brief Jacobian of h(x). Since this measurement is linear, just returns H.
    /// @param x State vector. 
    ///          [x, y, vx, vy] in constant-velocity model
    ///          [x, y, vx, vy, ax, ay] in constant-acceleration model
    /// @return H, where H is either
    ///         [1, 0, 0, 0]
    ///         [0, 1, 0, 0]
    ///         in the constant-velocity model or 
    ///         [1, 0, 0, 0, 0, 0]
    ///         [0, 1, 0, 0, 0, 0]
    ///         in the constant-acceleration model.
    Eigen::MatrixXd H(const Eigen::VectorXd& x) const override;
};


/// @brief Measurement model for a Lidar sensor.
struct LidarModel: public MeasurementModel{
    Eigen::Vector2d mast_position; // (x, y) position (world frame) of receiver mast

    /// @brief Measurement function for a lidar sensor.
    /// @param x State vector. 
    ///          [x, y, vx, vy] in constant-velocity model
    ///          [x, y, vx, vy, ax, ay] in constant-acceleration model
    /// @return h(x) = [|x - mast_position|, atan2((x - mast_position)(1), (x - mast_position)(0))]
    Eigen::VectorXd h(const Eigen::VectorXd& x) const override;


    /// @brief Jacobian matrix of h(x)
    /// @param x State vector. 
    ///          [x, y, vx, vy] in constant-velocity model
    ///          [x, y, vx, vy, ax, ay] in constant-acceleration model
    /// @return The matrix 
    ///         [(x - x_mast) / r  , (y - y_mast) / r  , 0, 0]
    ///         [(y_mast - y) / r^2, (x - x_mast) / r^2, 0, 0]
    Eigen::MatrixXd H(const Eigen::VectorXd& x) const override;


    /// @brief Innovation, except now we have to wrap the angle.
    /// @param z Measurement
    /// @param hx h applied to estimate
    /// @return Innovation z - hx, with the second coordinate (angle) wrapped
    Eigen::VectorXd residual(const Eigen::VectorXd& z, const Eigen::VectorXd& hx) const override;
};


#endif