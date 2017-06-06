#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


Tools::Tools() {};

Tools::~Tools() {};

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  // Initialize return
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check that input vectors are ok
  if(estimations.size() == 0 | estimations.size() != ground_truth.size()) {
    cout<<"Input for RMSE function is not good!";
    return rmse;
  }

  // Initialize residuals
  VectorXd residuals(4);

  for (int i=0; i<estimations.size(); i++) {

    residuals = estimations[i] - ground_truth[i];
    residuals = residuals.array() * residuals.array();

    rmse += residuals;
  }

  // Calculate mean of rmse vector
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &mu) {
  // Initialize return matrix
  MatrixXd Hj(3, 4);

  // extract state parameters: position p_ and velocities v_
  float p_x = mu(0);
  float p_y = mu(1);
  float v_x = mu(2);
  float v_y = mu(3);

  // calculate parameteres used several times
  float c1 = p_x * p_x + p_y * p_y;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  // check division by zero:
  if (fabs(c1) < 0.0001) {
    cout << "In Jacobian calculation a division by zero occured" << endl;
    return Hj;
  }

  // compute Jacobian
  Hj <<   (p_x/c2), (p_y/c2), 0, 0,
          (-p_y/c1), (p_x/c1), 0, 0,
          (p_y * (v_x * p_y - v_y * p_x)/c3), (p_x * (p_x * v_y - p_y * v_x)/c3), p_x/c2, p_y/c2;

  return Hj;


}