#ifndef TRUTH_STATE_H
#define TRUTH_STATE_H
#include <math.h>


struct TruthState{
    double t{0.0}; // s
    double x{0.0}; // m world frame
    double y{0.0}; // m world frame
    double theta{0.0}; // radians measured ccw from +x world frame.
    double speed{0.0}; // m/s imagine as being along track

    // Inputs from trajectory
    double omega{0.0}; // rad/s yaw rate
    double a_track{0.0}; // m/s^2 longitudinal acceleration

    // World frame velocity components
    double vx() const { return speed * std::cos(theta); }
    double vy() const { return speed * std::sin(theta); }

    // World frame acceleration components
    double ax() const { return a_track * std::cos(theta) - speed * omega * std::sin(theta); }
    double ay() const { return a_track * std::sin(theta) + speed * omega * std::cos(theta); }
};

#endif