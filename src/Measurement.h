#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <eigen3/Eigen/Core>
#include "SensorID.h"


struct Measurement{
    SensorID sensor_id;
    double t_generated {0.0};
    // Filter shouldn't be able to distinguish these, but simulation should be able to add delay between them.
    double t_arrived {0.0};
    Eigen::VectorXd z;
};


#endif
