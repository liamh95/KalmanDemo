#ifndef SENSOR_SIM_H
#define SENSOR_SIM_H

#include <eigen3/Eigen/Core>
#include <random>
#include <cmath>
#include "TruthState.h"
#include "SensorID.h"
#include "Measurement.h"
#include "Random.h"
#include "Angles.h"


struct SensorSim{
    SensorID sensor_id;
    double rate_hz;
    double latency {0.0};
    Eigen::MatrixXd L; // L * L^T = R, Cholesky.


    virtual Eigen::VectorXd h_true(const TruthState&) const = 0;


    /// @brief Overriding allows for angular measurements to wrap the angle.
    /// @param z Measurement vector
    /// @return The same measurement, possibly with the angle wrapped (if applicable).
    virtual Eigen::VectorXd canonicalize(Eigen::VectorXd z) const { return z ;}


    Measurement sample(const TruthState& s, std::mt19937& gen) const{
        return{
            sensor_id,
            s.t,
            s.t + latency,
            canonicalize(h_true(s) + L * sampleStdNormal(L.rows(), gen))
        };

    }


    virtual ~SensorSim() = default;
};


struct GPSSim : public SensorSim{
    Eigen::VectorXd h_true(const TruthState& s) const override { return Eigen::Vector2d {s.x, s.y}; }
};




struct LidarSim : public SensorSim{
    Eigen::Vector2d mast_position;


    Eigen::VectorXd h_true(const TruthState& s) const override{
        return Eigen::Vector2d { std::hypot(s.x - mast_position(0), s.y - mast_position(1)), std::atan2(s.y - mast_position(1), s.x - mast_position(0)) };
    }


    Eigen::VectorXd canonicalize(Eigen::VectorXd z) const override{
        z(1) = wrap_pi(z(1));
        return z;
    }
};

#endif