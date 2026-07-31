#ifndef ANGLES_H
#define ANGLES_H

#include <cmath>

constexpr double PI {3.14159265358979323846};
inline double wrap_pi(double a){ return std::remainder(a, 2.0 * PI); }
inline double angleDiff(double a, double b){ return wrap_pi(a - b); }

#endif