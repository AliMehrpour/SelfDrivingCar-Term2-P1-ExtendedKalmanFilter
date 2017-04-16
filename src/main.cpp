/*
 * main.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: Ali Mehrpour
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include "data_source.h"
#include "fusion_ekf.h"

using namespace std;

void check_arguments(int argc, char* argv[]) {
  string instruction = "Usage instructions: ";
  instruction += argv[0];
  instruction += " path/to/input.txt output.txt true|false(laser process flag) true|false(radar process flag)";

  bool valid = false;

  if (argc == 1) {
    cerr << instruction << endl;
  } else if (argc >= 2 && argc <= 4) {
    cerr << "Please include input file, output files and radar and laser process flags \n" << instruction;
  } else if (argc == 5) {
    valid = true;
  } else if (argc > 5) {
    cerr << "Too many arguments.\n" << instruction;
  }

  if (!valid) {
    exit(EXIT_FAILURE);
  }
}

void check_files(string in_file_name, string out_file_name) {
  ifstream in_file(in_file_name, ifstream::in);
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_file_name << endl;
    exit(EXIT_FAILURE);
  }

  ofstream out_file(out_file_name, ofstream::out);
  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_file_name << endl;
    exit(EXIT_FAILURE);
  }
}

void process(DataSource data_source, string out_file_name) {
  ofstream out_file(out_file_name, ofstream::out);

  FusionEKF fusionEKF;
  // Used to compute RMSE
  vector<VectorXd> estimations;
  vector<VectorXd> truths;

  size_t size = data_source.measurements_.size();
  for(size_t i = 0; i < size; ++i) {
    Data measurement = data_source.measurements_[i];
    Data truth = data_source.truths_[i];

    fusionEKF.ProcessMeasurement(measurement);

    VectorXd estimation = fusionEKF.ekf_.x_;

    // output the estimation
    out_file << estimation(0) << "\t";
    out_file << estimation(1) << "\t";
    out_file << estimation(2) << "\t";
    out_file << estimation(3) << "\t";

    // output the measurements
    if (measurement.sensor_type_ == Data::LASER) {
      // output the estimation
      out_file << measurement.data_(0) << "\t";
      out_file << measurement.data_(1) << "\t";
    }
    else if (measurement.sensor_type_ == Data::RADAR) {
      // output the estimation in the cartesian coordinates
      float rho = measurement.data_(0);
      float phi = measurement.data_(1);
      out_file << rho * cos(phi) << "\t";
      out_file << rho * sin(phi) << "\t";
    }

    // output the ground truth packages
    out_file << truth.data_(0) << "\t";
    out_file << truth.data_(1) << "\t";
    out_file << truth.data_(2) << "\t";
    out_file << truth.data_(3) << "\n";

    estimations.push_back(estimation);
    truths.push_back(truth.data_);
  }

  // Compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, truths) << endl;

  // close file
  if (out_file.is_open()) {
    out_file.close();
  }
}

int main(int argc, char* argv[]) {
  // Check arguments to make sure we get right arguments
  check_arguments(argc, argv);

  // Check files
  const string in_file_name = argv[1];
  const string out_file_name = argv[2];
  check_files(in_file_name, out_file_name);

  // Read data into DataSource object
  DataSource data_source;
  bool process_laser_measurement = (strcmp(argv[3], "true") == 0); // Set to true if laser measurements should be process, false otherwise
  bool process_radar_measurement = (strcmp(argv[4], "true") == 0); // Set to true if radar measurements should be process, false otherwise
  data_source.Load(in_file_name, process_laser_measurement, process_radar_measurement);
  cout << "Data load is complete." << endl;
  cout << data_source.ToString() << endl;

  // Process
  process(data_source, out_file_name);

  return 0;
}
