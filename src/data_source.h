/*
 * data_source.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Ali Mehrpour
 */

#ifndef DATA_SOURCE_H_
#define DATA_SOURCE_H_

#include <vector>
#include <iostream>
#include "data.h"

using namespace std;
using std::vector;

/**
 * Load measurement and ground truth data
 */
class DataSource {
public:
  vector<Data> measurements_; // Measurement list
  vector<Data> truths_; // Ground truth list

  DataSource();

  ~DataSource();

  /**
   * Load data from given file name
   */
  void Load(string file_name, bool process_laser_measurement, bool process_radar_measurement);

  /**
   * return string representation of current object
   */
  string ToString();
};

#endif /* DATA_SOURCE_H_ */
