#include <iostream>
#include <vector>
#include "measurement.h"
#include "fusion.h"
#include "tools.h"
#include "ekf.h"

EKF ekf;
Tools tools;
//Measurement measurement;

Fusion::Fusion() { is_initialized = false; }

Fusion::~Fusion() { }

void Fusion::init(const Measurement& measurement) {

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
    // predict based on the input and previous estimate
    ekf.Predict(); // updates P

    // vary update type 
    if(measurement.sensor_type==Measurement::LIDAR) {
        // LIDAR update
        ekf.UpdateLIDAR(measurement.raw);

    } else if(measurement.sensor_type==Measurement::RADAR) {
        // RADAR update
        ekf.H_RADAR  = tools.CalculateJacobian(ekf.x_hat);
        ekf.UpdateRADAR(measurement.raw);

    }
}


void Fusion::Process(const Measurement& measurement) {
    is_initialized ? this->Compute(measurement) : this->init(measurement);
}
