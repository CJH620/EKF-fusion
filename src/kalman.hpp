#include <Eigen/Dense>

#pragma once

class KalmanFilter {
public:
    KalmanFilter(
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& H,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& P,
            const Eigen::MatrixXd& R);

    // initializes states as zero
    void init();

    // initialize with first guess
    void init(const Eigen::VectorXd& x0);

    // prediction based on control input (time update)
    void predict(const Eigen::VectorXd& u);

    // update estimated state (measurement update)
    void update(const Eigen::VectorXd& z);

    // return current state
    Eigen::VectorXd state() { return x_hat; };

private:
    Eigen::MatrixXd A, B, H, Q, P, P0, R, K;

    // dims
    int m, n, l;

    bool initialized = false;

    // identity matrix
    Eigen::MatrixXd I;

    // estimated state
    Eigen::VectorXd x_hat;
};
