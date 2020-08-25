#ifndef EKF_H_
#define EKF_H_

#include <Eigen/Dense>

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

    void init();

    void Predict();

    void UpdateLIDAR(const Eigen::VectorXd& z);

    void UpdateRADAR(const Eigen::VectorXd& z);

private:
    Eigen::MatrixXd A, H_RADAR, H_LIDAR, Q, P, R_LIDAR, R_RADAR;

    int m, n, l;

    Eigen::MatrixXd I;

    Eigen::VectorXd x_hat;
};

#endif // EKF_H_
