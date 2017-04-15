/*
 * fusion_ekf.h
 *
 *  Created on: Apr 9, 2017
 *      Author: amehrpour
 */

#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_

#include "Eigen/Dense"
#include "data.h"
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Runs Extended Kalman Filter flow
 */
class FusionEKF {
public:
  KalmanFilter ekf_;

  FusionEKF();

  virtual ~FusionEKF();

  /**
   * Run Kalman Filter flow
   */
  void ProcessMeasurement(const Data &measurement);

private:
  bool is_initialized_; // Check whether FusionEKF was initialized or not
  long previous_timestamp_; // Previous timestamp
  Tools tools;

  MatrixXd R_laser_;
  MatrixXd R_radar_;
  MatrixXd H_laser_;
  MatrixXd Hj_;

  // Acceleration noise components
  float noise_ax_;
  float noise_ay_;
};

#endif /* FUSION_EKF_H_ */
