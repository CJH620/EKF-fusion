#include <iostream>
#include <vector>
#include "measurement.h"
#include "fusion.h"
#include "tools.h"

EKF ekf;
Tools tools;

Fusion::Fusion() {
    is_initialized = false;
    // initialize process and measurement noise values
}

Fusion::~Fusion() { }

void Fusion::init(const Measurement& measurement) {

    // set up initial values
    ekf.x_hat = Eigen::VectorXd(4);

    if(measurement.sensor_type==Measurement::LIDAR) {
        // if measurement type is LIDAR
        // dont have initial vx, vy so pack with zeros
        ekf.x_hat << measurement.raw[0], measurement.raw[1], 0, 0;

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
        ekf.x_hat << x, y, vx, vy

    }

    previous_timestamp = measurement.timestamp;
    is_initialized = true;

    Compute(measurement);
}

void Fusion::Compute(const Measurement& measurement) {
    // set up matrices
    // init H, R

    // predict based on the input and previous estimate
    ekf.predict(); // updates P

    // vary update type 
    if(measurement.sensor_type==Measurement::LIDAR) {
        // LIDAR update
        
        ekf.H = H_LIDAR;
        ekf.R = R_LIDAR;
        ekf.UpdateLIDAR(measurement.raw);

    } else if(measurement.sensor_type==Measurement::RADAR) {
        // RADAR update
        
        ekf.H = tools.CalculateJacobian(ekf.x_hat);
        ekf.R = R_RADAR;
        ekf.UpdateRADAR(measurement.raw);

    }

}


void Fusion::Process(const Measurement& measurement) {
    is_initialized ? this->Compute(measurement) : this->init(measurement);
}

double Fusion::GetEstimation() { return ekf.x_hat }

