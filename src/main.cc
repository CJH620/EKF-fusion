#include <iostream>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include <vector>

#include "fusion.h"
#include "ekf.h"
#include "ground_truth.h"
#include "measurement.h"
#include "tools.h"


int main(int argc, char* argv[]) {
    
    // the sensor data supplied as an argument
    std::string infile_name(argv[1]);
    // uses c-style string with newline instead of char*
    std::ifstream infile(infile_name.c_str());
    // each line in the input file
    std::string line;

    // initialize covariances
    int n = 3;
    int m = 1;
    int l = 1;

    Eigen::MatrixXd A(n,n);
    Eigen::MatrixXd B(n,l);
    Eigen::Matrixd H(m,n);
    Eigen::MatrixXd Q(n,n);
    Eigen::MatrixXd P(n,n);
    Eigen::MatrixXd R(m,m);

    // construct fusion and ekf instance
    Fusion fusion;
    EKF ekf;



    while(std::getline(infile, line)) {
        
        // constructed and destructed on each loop so each line has its own instance
        std::string sensor_type;
        Measurement measurement;
        GroundTruth groundtruth;
        long long timestamp;
    
        std::istringstream iss(line);

        // check sensor type in the first value
        iss >> sensor_type;

        if(sensor_type.compare("L")==0) {
            // LIDAR measurement

            // meas_px, meas_py
            float x, y;
            iss >> x;
            iss >> y;
            iss >> timestamp;
            // measurement class write
            measurement.raw << x, y;
            measurement.timestamp = timestamp;

        } else if(sensor_type.compare("R")==0) {
            // RADAR measurement

            float rho, phi, rho_dot;
            iss >> rho;
            iss >> phi;
            iss >> rho_dot;
            iss >> timestamp;
            // measurement class write
            measurement.raw << rho, phi, rho_dot;
            measurement.timestamp = timestamp;

        }

        // read ground truth data
        float gt_px, gt_py, gt_vx, gt_vy;
        iss >> gt_px;
        iss >> gt_py;
        iss >> gt_vx;
        iss >> gt_vy;


        // process measurements and set initial values/matrices
        fusion.Process(measurement);
        // predict the state ahead
        // project the error covariance ahead
        ekf.predict(u);
        // compute kalman gain
        // update measurement with z_k
        // update error covariance
        ekf.update(z);

        // est_px, est_py, est_vx, est_vy
        std::cout << ekf.state(0) << "," << ekf.state(1) << "," << ekf.state(2) << "," << ekf.state(3) << "," << measurement.raw[0] << "," << measurement.raw[1] << "," << gt_gx << "," << gt_gy << "," << gt_vx << "," << gt_vy << std::endl; 

        // destructors should auto call at out of scope? does it go out of scope?

    }
}



