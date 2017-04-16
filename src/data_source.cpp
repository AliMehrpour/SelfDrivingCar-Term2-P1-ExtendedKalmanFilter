/*
 * data_source.cpp
 *
 *  Created on: Apr 9, 2017
 *      Author: amehrpour
 */

#include <iostream>
#include <fstream>
#include "data_source.h"
#include "Eigen/Dense"

using namespace std;
using Eigen::VectorXd;

DataSource::DataSource() { }

DataSource::~DataSource() { }

void DataSource::Load(string file_name, bool process_laser_measurement, bool process_radar_measurement) {
  ifstream in_file(file_name, ifstream::in);
  string line;
  bool process_measurement;

  while (getline(in_file, line)) {
    istringstream iss(line);

    process_measurement = false;

    string sensor_type;
    Data measurement;
    Data truth;
    long long timestamp;

    // Read measurement
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0 && process_laser_measurement == true) {
      // LASER MEASUREMENT
      measurement.sensor_type_ = Data::LASER;

      float x;
      float y;
      iss >> x;
      iss >> y;
      measurement.data_ = VectorXd(2);
      measurement.data_ << x, y;

      iss >> timestamp;
      measurement.timestamp_ = timestamp;

      measurements_.push_back(measurement);

      process_measurement = true;
    }
    else if (sensor_type.compare("R") == 0 && process_radar_measurement == true) {
      // RADAR MEASUREMENT
      measurement.sensor_type_ = Data::RADAR;

      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      measurement.data_ = VectorXd(3);
      measurement.data_ << ro, phi, ro_dot;

      iss >> timestamp;
      measurement.timestamp_ = timestamp;

      measurements_.push_back(measurement);

      process_measurement = true;
    }

    if (process_measurement) {
      // Read ground truth
      float px;
      float py;
      float vx;
      float vy;
      iss >> px;
      iss >> py;
      iss >> vx;
      iss >> vy;
      truth.data_ = VectorXd(4);
      truth.data_ << px, py, vx, vy;

      truths_.push_back(truth);
    }
  }

  // Close file
  if (in_file.is_open()) {
    in_file.close();
  }
}

string DataSource::ToString() {
  string str = "Number of measurements :" ;
  str += to_string(measurements_.size());
  str += ", Number of ground truths: ";
  str += to_string(truths_.size());

  return str;
}
