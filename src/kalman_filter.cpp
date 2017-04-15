/*
 * kalman_filter.cpp
 *
 *  Created on: Apr 10, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() { }

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred; // Error calculation

  KF(y);
}

void KalmanFilter::updateEKF(const VectorXd &z) {
  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];

  // Convert polar to catesian coordinates
  double rho = sqrt((px*px) + (py*py));
  double phi = atan(py/px);
  double rho_dot = ((px*vx) + (py*vy)) / rho;

  MatrixXd h(3, 1); // h(x_)
  h << rho, phi, rho_dot;
  VectorXd y = z - h;

  KF(y);
}

void KalmanFilter::KF(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
