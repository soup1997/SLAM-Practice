#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include "eigen3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools
{
public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  MatrixXd CalculateJacobian(const VectorXd &x_state);

  /**
   * Noarmlize the angle
   */
  float normalize_angle(float phi);

  void normalize_bearing(VectorXd &Z);
};

#endif /* TOOLS_H_ */
