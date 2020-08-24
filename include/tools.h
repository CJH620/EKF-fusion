#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <Eigen/Dense>

class Tools {
public:
    
    Tools();

    ~Tools();

    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_hat);

private:
    Eigen::MatrixXd Hj;
};

#endif // TOOLS_H_
