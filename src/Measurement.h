#include <eigen3/Eigen/Core>


enum SensorType{

    IMU,
    GPS,
    Lidar,
};


struct Measurement{
    SensorType sensorID;
    double t_generated {0.0};
    // Filter shouldn't be able to distinguish these? But simulation should be able to add delay between them?
    double t_arrived {0.0};
    Eigen::VectorXd z;
};


struct MeasurementModel{
    Eigen::MatrixXd R; // observation noise covariance
    
};