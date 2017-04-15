/*
 * tools.cpp
 *
 *  Created on: Apr 10, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include <vector>
#include "tools.h"

using namespace std;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &truths) {
  VectorXd rsme(4);
  rsme << 0, 0, 0, 0;

  int size_estimation = estimations.size();
  int size_truth = truths.size();

  if (size_estimation == 0) {
    cout << "Estimation vector is empty";
    return rsme;
  }

  if (size_estimation != size_truth) {
    cout << "Size of estimation and ground truth vectors are different";
    return rsme;
  }

  for(int i = 0; i < size_estimation; ++i) {
    VectorXd residual = estimations[i] - truths[i];
    residual = residual.array() * residual.array();
    rsme = rsme + residual;
  }

  rsme = (rsme.array() / size_estimation).array().sqrt();

  return rsme;
}


MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if (fabs(px) < EPS && fabs(py) < EPS) {
    px = EPS;
    py = EPS;
  }

  float c1 = px*px + py*py;

  // Check for division by zero
  if (fabs(c1) < EPS2) {
    c1 = EPS2;
  }

  float c2 = sqrt(c1);
  float c3 = (c1 * c2);

  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
        (py*(vx*py - vy*px) /c3), (px*(px*vy - py*vx)/c3), (px/c2), (py/c2);

  return Hj;
}

