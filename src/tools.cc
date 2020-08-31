#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include "tools.h"

#define EPS 0.0000001

Tools::Tools() { }

Tools::~Tools() { }

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_hat) {
    float px = x_hat(0);
    float py = x_hat(1);
    float vx = x_hat(2);
    float vy = x_hat(3);
    Hj = Eigen::MatrixXd(3,4);

    float c1 = px * px + py * py;
    
    if(abs(c1) < EPS) { c1 = EPS; }

    float c2 = sqrt(c1);
    float c3 = c1 * c2;

    Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
       py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
}
