#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {};

KalmanFilter::~KalmanFilter() {};

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &Q_in) {
  x_ = x_in;

  P_ = P_in;

  Q_ = Q_in;

  // Initialize laser matrices
  MatrixXd H_laser_in(2, 4);
  H_laser_in <<
             1, 0, 0, 0,
          0, 1, 0, 0;

  H_laser_ = H_laser_in;

  MatrixXd R_laser_in(2, 2);
  R_laser_in <<
             0.0225, 0,
          0, 0.0225;

  R_laser_ = R_laser_in;

  // Initialize radar matrix
  MatrixXd R_radar_in(3, 3);

  R_radar_in << 0.09, 0, 0,
          0, 0.0009, 0,
          0, 0, 0.09;

  R_radar_ = R_radar_in;

}

void KalmanFilter::Predict(float dt) {
  // Calculate transition matrix
  MatrixXd F_(4, 4);
  F_ <<
     1, 0, dt, 0,
          0, 1, 0, dt,
          0, 0, 1, 0,
          0, 0, 0, 1;

  x_ = F_ * x_;

  // Calculate process noise
  MatrixXd Q_process_full(4, 4);
  MatrixXd G(4, 2);
  float dt_squared = dt * dt / 2;
  G << dt_squared, 0,
          0, dt_squared,
          dt, 0,
          0, dt;

  Q_process_full = G * Q_ * G.transpose();

  // Todo calculate updated covariance matrix!
  P_ = F_ * P_ * F_.transpose() + Q_process_full;
};

void KalmanFilter::Update(const VectorXd &z) {

  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new estimate
  x_ = x_ + (K * y);
  long n = x_.size();
  MatrixXd I = MatrixXd::Identity(n, n);
  P_ = (I - K * H_laser_) * P_;
};

void KalmanFilter::UpdateRadar(const VectorXd &z) {
  // Calculate polar coordinates
  float rho = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  float phi = atan(x_(1) / x_(0));
  float rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

  // Throw polar coordinates in vector
  VectorXd x_polar(3);
  x_polar << rho, phi, rho_dot;

  std::cout << "Radar measurement" << std::endl;
  std::cout << z << std::endl << std::endl;
  std::cout << "State vector in polar coordinates" << std::endl << std::endl;
  std::cout << x_polar << std::endl << std::endl;

  // Get Jacobian
  MatrixXd Hj(3, 4);
  Tools tools;
  Hj = tools.CalculateJacobian(x_);


  // Calculations as for laser
  VectorXd y = z - x_polar;
  if (y(1) > M_PI / 2) {
    std::cout << "FUFUFU";
    y(1) -= M_PI;
  } else if (y(1) < -M_PI / 2) {
    std::cout << "DUDUDU";
    y(1) += M_PI;
  }
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new estimate
  x_ = x_ + (K * y);
  long n = x_.size();
  MatrixXd I = MatrixXd::Identity(n, n);
  P_ = (I - K * Hj) * P_;
}