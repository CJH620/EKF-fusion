#ifndef EKF_H_
#define EKF_H_

#include <Eigen/Dense>
#include "measurement.h"
#include "tools.h"
#include "kalman.h"

class EKF {
public:
    EKF();

    virtual ~EKF();

    void Process(const Measurements &measurement_pack);

    KalmanFilter ekf;

private:
    bool initialized;

    long previous_timestamp;

    // probs should initialize tools here

    Eigen::MatrixXd R_lidar;
    Eigen::MatrixXd R_radar;
    Eigen::MatrixXd H_lidar;
    Eigen::MatrixXd Hj;
};

#endif //EKF_H_
