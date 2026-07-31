#include "Trajectory.h"



TruthState Trajectory::sample(const double t) const{
    if(samples.empty())
        return TruthState{};

    if(t <= samples.front().t)
        return samples.front();

    if(t >= samples.back().t)
        return samples.back();

    const std::size_t i = std::min(static_cast<std::size_t>((t - samples.front().t) / dt), samples.size() - 2);
    const TruthState& prev = samples[i];
    const TruthState& next = samples[i+1];
    const double w = (t - prev.t) / dt;

    auto lerp = [w](const double a, const double b){ return a + w*(b-a); };

    return TruthState{
        t,
        lerp(prev.x, next.x),
        lerp(prev.y, next.y),
        prev.theta + w*angleDiff(next.theta, prev.theta),// wrap?
        lerp(prev.speed, next.speed),
        lerp(prev.omega, next.omega),
        lerp(prev.a_track, next.a_track)
    };
}

/// @brief Computes time derivative of state vector x = [x, y, theta, speed]
/// @param s State vector [x, y, theta, speed]
/// @param omega Angular velocity rad/s
/// @param a_track On-track acceleration m/s^2
/// @return [speed * cos theta, speed * sin theta, omega, a_track]
Eigen::Vector4d deriv(const Eigen::Vector4d& s, const double omega, const double a_track){
    const double theta = s(2);
    const double speed = s(3);
    return {speed * std::cos(theta), speed * std::sin(theta), omega, a_track};
}


Trajectory buildTrajectory(const TrajectoryProfile& profile, const double duration, const double dt, std::mt19937& gen){
    Trajectory traj;
    traj.dt = dt;
    traj.duration = duration;

    // number of points
    const std::size_t n = static_cast<std::size_t>(duration / dt) + 1;
    traj.samples.reserve(n);

    std::normal_distribution<double> std_normal(0.0, 1.0);
    const double omega_noise_scale = profile.sigma_omega / std::sqrt(dt);
    const double a_noise_scale = profile.sigma_a_track / std::sqrt(dt);

    // [x, y, theta, speed]
    Eigen::Vector4d s {profile.x0, profile.y0, profile.theta0, profile.speed0};

    for(std::size_t i = 0; i < n; i++){
        const double t = i * dt;
        const double omega = profile.omega(t) + omega_noise_scale * std_normal(gen);
        const double a_track = profile.a_track(t) + a_noise_scale * std_normal(gen);

        traj.samples.push_back(TruthState{t, s(0), s(1), s(2), s(3), omega, a_track});

        // RK4
        const Eigen::Vector4d k1 = deriv(s, omega, a_track);
        const Eigen::Vector4d k2 = deriv(s + 0.5 * dt * k1, omega, a_track);
        const Eigen::Vector4d k3 = deriv(s + 0.5 * dt * k2, omega, a_track);
        const Eigen::Vector4d k4 = deriv(s + dt * k3, omega, a_track);
        s += (dt / 6) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    }
    return traj;
}
