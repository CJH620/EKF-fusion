#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <Eigen/Dense>

class Measurements {
    public:
        enum SensorType {
            LIDAR,
            RADAR
        } sensor_type_;

        long long timestamp_;

        Eigen::VectorXd raw_measurements_;
};

#endif // MEASUREMENT_H_
