#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include "ekf.h"
#include "tools.h"

EKF::EKF(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& H_LIDAR,
        const Eigen::MatrixXd& H_RADAR,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& P,
        const Eigen::MatrixXd& R_LIDAR,
        const Eigen::MatrixXd& R_RADAR) { }
        
EKF::~EKF() { }

// matrix initialization is done within fusion.cc due to sensor_type
void EKF::init() { }

void EKF::predict() {
    x_hat = A * x_hat;
    P = A * P * A.transpose() + Q; 
}

void EKF::updateLIDAR(const Eigen::VectorXd& z) {
    // LIDAR raw values are x, y  so its easy
    K = P * H.transpose() + (H * P * H.transpose() + R).inverse();
    x_hat += K*(z - H * x_hat);
    P = (I - K * H) * P;
}

void EKF::updateRADAR(const Eigen::VectorXd& z) {
    // RADAR raw measurements are rho, phi, rho_dot
    double rho = sqrt(pow(x_hat(0), 2) + pow(x_hat(1), 2));
    double theta = atan(x_hat(1) / x_hat(0));
    double rho_dot = (x_hat(0) * x_hat(2) + x_hat(1) * x_hat(3)) / rho;
    h << rho, theta, rho_dot;

    K = P * H.transpose() + (H * P * H.transpose() + R).inverse();
    x_hat += K*(z - h);
    P = (I - K * H) * P;
} 

