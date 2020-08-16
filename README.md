# Extended Kalman Filter for noisy lidar and radar measurements

An EKF in C++ for fusing noisy lidar and radar sensor measurements. Lidar sensor returns position in cartesian coordinates (x, y) and the radar sensor returns position and relative velocity in polar coordinates (rho, phi, drho). The goal is to predict the state vector (position and velocity) at any point in time. 

EKF differs from a linear discrete KF in that it deals with non-linear processes by linearizing the current mean and covariance. A flaw is that the distributions of the random variables are no longer normal after undergoing nonlinear transformations. Thus the EKF is simply an ad hoc state estimator that only approximates the optimality of Bayes' rule by linearization. An important feature is the Jacobian in the equation for the Kalman gain services to correctly propagate only the portion of the residual that does affect the state. 

The input data is in the format:
* #L (lidar) meas_px meas_py timestamp gt_px gt_py gt_vx gt_vy
* #R (radar) meas_rho meas_phi meas_rho_dot timestamp gt_px gt_py gt_vx gt_vy

The output data is in the format:
* est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy


## file structure



## dependencies
* cmake 3.18.1
* make 3.18
* gcc 10.2

## usage

`./run.sh`


## resources
* https://github.com/udacity/CarND-Extended-Kalman-Filter-Project
* https://courses.cs.washington.edu/courses/cse571/03wi/notes/welch-bishop-tutorial.pdf


## todo
* port to [carla](https://github.com/carla-simulator/carla) -> should use linux. automated build on mac isnt set up yet. there is a PR from geohot [here](https://github.com/carla-simulator/carla/pull/2433) so maybe its possible soon.
