#ifndef FUSION_H_
#define FUSION_H_

#include <Eigen/Dense>
#include "measurement.h"
#include "ekf.h"

EKF ekf;
Measurement measurement;

class Fusion {
public:
    Fusion();

    ~Fusion();

    void init(const Measurement& measurement);

    void Compute(const Measurement& measurement);

    void Process(const Measurement& measurement);

    Eigen::VectorXd State() { return ekf.x_hat; }; 

private:
    bool is_initialized;
    double previous_timestamp;

};

#endif // FUSION_H_
