#include <iostream>

#include "kalman.hpp"

KalmanFilter::KalmanFilter(
        const Eigen::MatrixXd& A, // relates state at k-1 to k
        const Eigen::MatrixXd& B, // relates optimal control input to x
        const Eigen::MatrixXd& H, // relates state to measurement z
        const Eigen::MatrixXd& Q, // process noise covariance
        const Eigen::MatrixXd& P,
        const Eigen::MatrixXd& R) // measurement noise covariance
    : A(A), B(B), H(H), Q(Q), P0(P), R(R),
    m(H.rows()), n(A.rows()), l(B.cols()), initialized(false),
    I(n,n), x_hat(n)
{
    I.setIdentity();
}

void KalmanFilter::init(const Eigen::VectorXd& x0) {
    x_hat = x0;
    P = P0;
    initialized = true;
}

void KalmanFilter::init() {
    x_hat.setZero();
    P = P0;
    initialized = true;
}


void KalmanFilter::predict(const Eigen::VectorXd& u) {

    if (!initialized) {
        std::cout << "Not initialized. Initializing at origin." << std::endl;
        init();
    }

    // project state ahead
    x_hat = A * x_hat + B * u;
    // project error covarianceahead
    P = A * P * A.transpose() + Q;

}

void KalmanFilter::update(const Eigen::VectorXd& z) {

    // compute kalman gain K
    K = P * H.transpose() * (H*P*H.transpose() + R).inverse();
    // measure process and generate an a posteriori state
    x_hat = x_hat + K*(z - H * x_hat);
    // obtain an a posteriori error covariance
    P = (I - K * H) * P;

}
