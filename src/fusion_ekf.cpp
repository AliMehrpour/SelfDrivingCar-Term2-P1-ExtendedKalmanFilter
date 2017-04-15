/*
 * fusion_ekf.cpp
 *
 *  Created on: Apr 12, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include "fusion_ekf.h"
#include "tools.h"
#include "data.h"

using namespace std;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // Laser sensor measurement convariance matrix
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0255;

  // Radar sensor measurement convariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Laser sensor measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Radar sensor Jacobian matrix
  Hj_ = MatrixXd(3, 4);

  ekf_ = KalmanFilter();

  // State covariance matrix.
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  // Transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Process noise
  noise_ax_ = 9.0;
  noise_ay_ = 9.0;
}

FusionEKF::~FusionEKF() { }

void FusionEKF::ProcessMeasurement(const Data &measurement) {
  // Step 1: Initialization
  if (!is_initialized_) {
    cout << "Initializing Fusion Extended Kalman Filter" << endl;

    float px, py, vx, vy;
    if (measurement.sensor_type_ == Data::LASER) {
      px = measurement.data_[0];
      py = measurement.data_[1];
      vx = 0;
      vy = 0;
    }
    else if (measurement.sensor_type_ == Data::RADAR) {
      // Convert polar to cartesian coordinates
      float rho = measurement.data_[0];
      float phi = measurement.data_[1];
      float rho_dot = measurement.data_[2];
      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rho_dot * cos(phi);
      vy = rho_dot * sin(phi);
    }

    // Handling special cases initialization issues
    if (fabs(px) < EPS and fabs(py) < EPS) {
      px = EPS;
      py = EPS;
    }

    // Initialize ekf_.x_ with first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << px, py, vx, vy;

    is_initialized_ = true;
    previous_timestamp_ = measurement.timestamp_;
    return;
  }

  // Step 2: Predict
  // Compute the elapsed time between current and previous measurements
  long timestamp = measurement.timestamp_;
  float dt = (timestamp - previous_timestamp_) / 1000000.0; // seconds
  previous_timestamp_ = timestamp;

  // Update transition matrix F with delta time
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Set process covariance
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float dt_4_4 = dt_4 / 4;
  float dt_3_2 = dt_3 / 2;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << (dt_4_4 * noise_ax_), 0, (dt_3_2 * noise_ax_), 0,
             0, (dt_4_4 * noise_ay_), 0, (dt_3_2 * noise_ay_),
             (dt_3_2 * noise_ax_), 0, (dt_2 * noise_ax_), 0,
             0, (dt_3_2 * noise_ay_), 0, (dt_2 * noise_ay_);

  ekf_.Predict();

  // Step 3: Update
  if (measurement.sensor_type_ == Data::LASER) {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement.data_);
  }
  else if (measurement.sensor_type_ == Data::RADAR) {
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.updateEKF(measurement.data_);
  }

  cout << "x_ =" << ekf_.x_ << endl;
  cout << "P_ =" << ekf_.P_ << endl;
}
