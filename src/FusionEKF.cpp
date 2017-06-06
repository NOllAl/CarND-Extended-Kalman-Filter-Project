#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices

  //measurement covariance matrix - laser
  Eigen::MatrixXd R_laser_in(2, 2);
  R_laser_in <<
             0.0225, 0,
             0, 0.0225;

  R_laser_ = R_laser_in;

  //measurement covariance matrix - radar
  Eigen::MatrixXd R_radar_in(3, 3);

  R_radar_in << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  R_radar_ = R_radar_in;

  Eigen::MatrixXd Q_in(2, 2);
  Q_in <<
       9, 0,
       0, 9;

  Q_ = Q_in;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    Eigen::VectorXd x_init(4);
    x_init << 1, 1, 1, 1;
    Eigen::MatrixXd p_init(4, 4);
    p_init << 1000, 0, 0, 0,
            0, 1000, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    ekf_.Init(x_init, p_init, Q_);


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      Eigen::VectorXd measurement_radar(3);
      measurement_radar = measurement_pack.raw_measurements_;
      ekf_.UpdateRadar(measurement_radar);
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << measurement_pack.raw_measurements_;
      Eigen::VectorXd measurement_laser(2);
      measurement_laser = measurement_pack.raw_measurements_;
      ekf_.Update(measurement_laser);
      /**
      Initialize state.
      */
    }

    // Set right timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  std::cout << dt << "\n\n\n";
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Eigen::VectorXd measurement_radar(3);
    measurement_radar = measurement_pack.raw_measurements_;
    ekf_.UpdateRadar(measurement_radar);
  } else {
    // Laser updates
    Eigen::VectorXd measurement_laser(2);
    measurement_laser = measurement_pack.raw_measurements_;
    ekf_.Update(measurement_laser);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
