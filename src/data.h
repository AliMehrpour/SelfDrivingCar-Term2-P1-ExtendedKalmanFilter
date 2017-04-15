/*
 * data.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Ali Mehrpour
 */

#ifndef DATA_H_
#define DATA_H_

#include "Eigen/Dense"

/**
 * Represent data row in data source file. Data can be measurement or ground truth
 */
class Data {
public:
  long long timestamp_;
  Eigen::VectorXd data_;
  enum SensorType {
    LASER, RADAR
  } sensor_type_;
};

#endif /* DATA_H_ */
