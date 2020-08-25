#include <iostream>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>
#include "fusion.h"
#include "measurement.h"


int main(int argc, char* argv[]) {
    
    // implement checks for file presence


    // the sensor data supplied as an argument
    std::string infile_name(argv[1]);
    // uses c-style string with newline instead of char*
    std::ifstream infile(infile_name.c_str());
    // each line in the input file
    std::string line;

    // initialize covariances
    int n = 4;
    int m = 3;
    int l = 2;

    Eigen::MatrixXd A(n,n);
    Eigen::MatrixXd H_LIDAR(l,n);
    Eigen::MatrixXd Q(n,n);
    Eigen::MatrixXd P(n,n);
    Eigen::MatrixXd R_LIDAR(l,l);
    Eigen::MatrixXd R_RADAR(m,m);

    double dt = 1.0;

    // init A, H_LIDAR, H_RADAR, Q, P, R_LIDAR, R_RADAR
    A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

    H_LIDAR << 1, 0, 0, 0, 0, 1, 0, 0;

    Q << .05, 0, .05, 0, 0, .05, 0, .05, .05, 0, .05, 0, 0, .05, 0, .05;

    P << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0,0, 0, 0, 1000;

    R_LIDAR << .0225, 0, 0, .0225;

    R_RADAR << .09, 0, 0, 0, .0009, 0, 0, 0, .09;

    Fusion fusion(A, H_LIDAR, Q, P, R_LIDAR, R_RADAR);

    while(std::getline(infile, line)) {
    
        // constructed and destructed on each loop so each line has its own instance
        std::string sensor_type;
        Measurement measurement;
        long long timestamp;
        Eigen::VectorXd estimation;
    
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
        // predict the state ahead, project the error covariance
        // compute kalman gain, update measurement with zK, update error covariance
        fusion.Process(measurement);
//        estimation.push_back(fusion.GetEstimation());

        std::cout << fusion.state()[0] << "," << fusion.state()[1] << "," << fusion.state()[2] << "," << fusion.state()[3] << "," << measurement.raw[0] << "," << measurement.raw[1] << "," << gt_gx << "," << gt_gy << "," << gt_vx << "," << gt_vy << std::endl; 

        // destructors should auto call at out of scope? does it go out of scope?

    }

    if(infile.is_open()) { infile.close(); }

    return 0;
}
