#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <functional>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <random>
#include <eigen3/Eigen/Core>
#include "TruthState.h"
#include "Angles.h"

struct TrajectoryProfile{
    std::function<double(double)> omega; // rad/s yaw rate
    std::function<double(double)> a_track; // m/s^2 on-track acceleration

    double sigma_omega {0.0}; // rad / sqrt(s) so per-step perturbation sigma_omega / sqrt(t) gives rad / s
    double sigma_a_track {0.0}; // m / s^(3/2) so per-step perturbation sigma_a_track / sqrt(t) gives m / s^2

    double x0 {0.0}; // m initial x
    double y0 {0.0}; // m initial y
    double theta0 {0.0}; // rad initial heading
    double speed0 {0.0}; // m/s initial speed
};


struct Trajectory{
    std::vector<TruthState> samples;
    double dt {0.0};
    double duration {0.0};

    /// @brief Returns the true state of the system at the specified time on the trajectory.
    ///        Linearly interpolates between the states at the two closest time steps.
    /// @param t Time at which we want the state.
    /// @return Linear interpolation of the states at the two closest time steps.
    TruthState sample(const double t) const;
};

/// @brief Builds a trajectory with process noise (a vector<TruthState>) by integrating the profile with RK4.
/// @param profile Specifies yaw rate and on-track acceleration functions, their process noise std deviations
///                and initial conditions.
/// @param duration Duration of trajectory in seconds.
/// @param dt Time step in seconds.
/// @param gen Random number generator.
/// @return 
Trajectory buildTrajectory(const TrajectoryProfile& profile, const double duration, const double dt, std::mt19937& gen);

#endif