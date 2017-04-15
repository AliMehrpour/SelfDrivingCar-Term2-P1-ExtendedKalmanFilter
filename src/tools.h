/*
 * tools.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Ali Mehrpour
 */

#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

#define EPS 0.0001
#define EPS2 0.000001

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Tools {
public:
  /**
   * Calculate Root Mean Square Error(RMSE) of estimations and truth values
   */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &truths);

  /**
   * Calculate Jacobian matrix
   */
  MatrixXd CalculateJacobian(const VectorXd &x_state);
};

#endif /* TOOLS_H_ */
