#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include "Eigen/Dense"
#include <math.h>
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
    // mean of state
    VectorXd x_;

    // covariance matrix
    MatrixXd P_;

    // Process noise matrix
    MatrixXd Q_;

    // Laser measurement matrix
    MatrixXd H_laser_;

    // Laser measurement covariance matrix
    MatrixXd R_laser_;

    // Radar measurement covariance matrix
    MatrixXd R_radar_;

    // Constructor
    KalmanFilter();

    // Destructor
    virtual ~KalmanFilter();

    /**
     * Initialization method
     * @param x_in Initial mean state vector
     * @param P_in Initial state vector
     * @param Q_in Process noise matrix
     */
    void Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &Q_in);

    /**
     * Predict method to calculate new state given a time difference dt
     * @param dt time difference between last measurements and current measurement
     */
    void Predict(float dt);

    /**
     * Update method for Laser measurement: takes in the laser measurement vector and updates to new state
     * @param z the measurement at the next timestep
     */
    void Update(const VectorXd &z);

    /**
     * Update method for Radar measurement: takes in the radar measurement and updates to new state
     * @param z the measurement at the next timestep
     */
    void UpdateRadar(const VectorXd &z);
};

#endif