#include "MeasurementModel.h"
#include <cmath>


Eigen::VectorXd MeasurementModel::residual(const Eigen::VectorXd& z, const Eigen::VectorXd& hx) const{
    return z - hx;
}


Eigen::MatrixXd GPSModel::H(const Eigen::VectorXd& x) const{
    Eigen::MatrixXd Hx = Eigen::MatrixXd::Zero(2, x.size());
    Hx.topLeftCorner<2, 2>().setIdentity();
    return Hx;
}


Eigen::VectorXd GPSModel::h(const Eigen::VectorXd& x) const{
    return x.head<2>();
}


Eigen::VectorXd LidarModel::h(const Eigen::VectorXd& x)const{
    Eigen::Vector2d diff = x.head<2>() - mast_position;
    return Eigen::Vector2d { diff.norm(), std::atan2(diff(1), diff(0)) };
}


Eigen::MatrixXd LidarModel::H(const Eigen::VectorXd& x) const{
    Eigen::Vector2d diff = x.head<2>() - mast_position;
    const double r = diff.norm();
    Eigen::MatrixXd Hx = Eigen::MatrixXd::Zero(2, x.size());
    Hx.topLeftCorner<2, 2>() =  Eigen::Matrix<double, 2, 2> { {diff(0) / r, diff(1) / r}, {-1 * diff(1) / (r * r), diff(0) / (r * r)}  };
    return Hx;
}


Eigen::VectorXd LidarModel::residual(const Eigen::VectorXd&z, const Eigen::VectorXd& hx) const{
    Eigen::Vector2d y = z - hx;
    y(1) = wrap_pi(y(1));
    return y;
}