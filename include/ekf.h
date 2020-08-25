#ifndef EKF_H_
#define EKF_H_

#include <Eigen/Dense>
#include "measurement.h"

class EKF {
public:
    EKF(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& H_LIDAR,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& P,
        const Eigen::MatrixXd& R_LIDAR,
        const Eigen::MatrixXd& R_RADAR);

    ~EKF();

    void init(const Measurement& measurement);

    void Compute(const Measurement& measurement);    

    void Process(const Measurement& measurement);

    Eigen::VectorXd State() { return x_hat; }; 

private:
    bool is_initialized;
    int m, n, l;

    Eigen::VectorXd h;
    Eigen::MatrixXd I;
    Eigen::VectorXd x_hat;
    Eigen::MatrixXd A, H_RADAR, H_LIDAR, Q, P, R_LIDAR, R_RADAR, K;
};

#endif // EKF_H_
