#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "libs/Eigen/Dense"

namespace utility
{
  enum SensorType{
    LASER,
    RADAR
  };

  struct SensorReading {
    long long timestamp;
    SensorType sensor_type;
    Eigen::VectorXd measurement;
  };

  const Eigen::VectorXd CalculateRmse(const std::vector<Eigen::VectorXd> &estimations,
                                      const std::vector<Eigen::VectorXd> &ground_truth);
  const Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  const Eigen::VectorXd PolarToCartesian(const Eigen::VectorXd& polar_vector);
  const Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& x_state);

  void CheckArguments(int argc, char* argv[]);
  void CheckFiles(std::ifstream& in_file, std::string& in_name,
                  std::ofstream& out_file, std::string& out_name);
};

#endif /* UTILITY_H_ */
