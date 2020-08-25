#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include "ekf.h"

EKF::EKF(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& H_LIDAR,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& P,
        const Eigen::MatrixXd& R_LIDAR,
        const Eigen::MatrixXd& R_RADAR) { }
        
EKF::~EKF() { }

// matrix initialization is done within fusion.cc due to sensor_type
void EKF::init() { }

void EKF::Predict() {
    x_hat = A * x_hat;
    P = A * P * A.transpose() + Q; 
}

void EKF::UpdateLIDAR(const Eigen::VectorXd& z) {
    // LIDAR raw values are x, y  so its easy
    K = P * H_LIDAR.transpose() + (H_LIDAR * P * H_LIDAR_LIDAR.transpose() + R_LIDAR).inverse();
    x_hat += K*(z - H_LIDAR * x_hat);
    P = (I - K * H_LIDAR) * P;
}

void EKF::UpdateRADAR(const Eigen::VectorXd& z) {
    // RADAR raw measurements are rho, phi, rho_dot
    double rho = sqrt(pow(x_hat(0), 2) + pow(x_hat(1), 2));
    double theta = atan(x_hat(1) / x_hat(0));
    double rho_dot = (x_hat(0) * x_hat(2) + x_hat(1) * x_hat(3)) / rho;
    h << rho, theta, rho_dot;

    K = P * H_RADAR.transpose() + (H_RADAR * P * H_RADAR.transpose() + R_RADAR).inverse();
    x_hat += K*(z - h);
    P = (I - K * H_RADAR) * P;
} 

