#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include "measurement.h"
#include "tools.h"
#include "ekf.h"

Tools tools;

EKF::EKF(
        const Eigen::MatrixXd& A,
        const Eigen::MatrixXd& H_LIDAR,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& P,
        const Eigen::MatrixXd& R_LIDAR,
        const Eigen::MatrixXd& R_RADAR) 
    : A(A), H_LIDAR(H_LIDAR), Q(Q), P(P), R_LIDAR(R_LIDAR), R_RADAR(R_RADAR), m(R_RADAR.rows()), n(Q.rows()), l(R_LIDAR.rows()), I(n,n), x_hat(n) {
    is_initialized = false;
}

EKF::~EKF() { }

void EKF::init(const Measurement& measurement) {

    if(measurement.sensor_type==Measurement::LIDAR) {
        // if measurement type is LIDAR
        // dont have initial vx, vy so pack with zeros
        x_hat << measurement.raw[0], measurement.raw[1], 0, 0;

    } else if(measurement.sensor_type==Measurement::RADAR) {
        // if measurement type is RADAR
        // need to convert the polar values to cartesian
        double rho = measurement.raw[0];
        double phi = measurement.raw[1];
        double rho_dot = measurement.raw[2];

        double x = rho * cos(phi);
        double y = rho * sin(phi);
        double vx = rho_dot * cos(phi);
        double vy = rho_dot * sin(phi);
        x_hat << x, y, vx, vy;

    }

    is_initialized = true;

    Compute(measurement);
}

void EKF::Compute(const Measurement& measurement) {
    // no need for extra functions for update and predict, just put em here

    // kalman filter predict
    x_hat = A * x_hat;
    P = A * P * A.transpose() + Q;

    // vary update type 
    if(measurement.sensor_type==Measurement::LIDAR) {
        // LIDAR update
        // normal kalman filter update

        K = P * H_LIDAR.transpose() + (H_LIDAR * P * H_LIDAR.transpose() + R_LIDAR).inverse();
        x_hat += K*(measurement.raw - H_LIDAR * x_hat);
        P = (I - K * H_LIDAR) * P;

    } else if(measurement.sensor_type==Measurement::RADAR) {
        // RADAR update
        // slightly modified because measurement input varies

        double rho = sqrt(x_hat(0) * x_hat(0) + x_hat(1) * x_hat(1));
        double theta = atan(x_hat(1) / x_hat(0));
        double rho_dot = (x_hat(0) * x_hat(2) + x_hat(1) * x_hat(3)) / rho;
        h << rho, theta, rho_dot;
        
        H_RADAR = tools.CalculateJacobian(x_hat);

        K = P * H_RADAR.transpose() + (H_RADAR * P * H_RADAR.transpose() + R_RADAR).inverse();
        x_hat += K*(measurement.raw - h);
        P = (I - K * H_RADAR) * P;
    }
}


void EKF::Process(const Measurement& measurement) {
    is_initialized ? this->Compute(measurement) : this->init(measurement);
}
