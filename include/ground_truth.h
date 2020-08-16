#ifndef GROUND_TRUTH_H_
#define GROUND_TRUTH_H_

#include <Eigen/Dense>

class GroundTruth {
public:
    long long timestamp;

    enum SensorType {
        LIDAR,
        RADAR
    } sensor_type;

    Eigen::VectorXd values;
};

#endif // GROUND_TRUTH_
