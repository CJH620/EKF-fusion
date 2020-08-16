#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "measurement.h"
#include "fusion.h"
#include "tools.h"

Fusion::Fusion() {
    is_initialized = false;
    // initialize process and measurement noise values
}

// destroy
Fusion::~Fusion() {}

void Fusion::init(const Measurement& measurement) {

    // set up initial values
    ekf.state = Eigen::VectorXd(4);

    if(measurement.sensor_type==Measurement::LIDAR) {
        // if measurement type is LIDAR
        // dont have initial vx, vy so pack with zeros
        ekf.state << measurement.raw[0], measurement.raw[1], 0, 0;

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
        ekf.state << x, y, vx, vy

    }

    // debugging
    std::cout << ekf.state << std::endl;

    previous_timestamp = measurement.timestamp;
    is_initialized = true;
}

void Fusion::Compute(const Measurement &measurement) {
    




}




void Fusion::Process(const Measurement& measurement) {
    is_initialized ? this->Compute(measurement) : this->init(measurement);
}
