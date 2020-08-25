#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <Eigen/Dense>

class Measurement {
public:
    long long timestamp;

    enum SensorType {
        LIDAR,
        RADAR
    } sensor_type;

    Eigen::VectorXd raw_measurements;
};

#endif // MEASUREMENT_H_
