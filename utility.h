#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include "libs/Eigen/Dense"

namespace utility
{
  //These typedefs make it a cleaner API for the rest of the codebase
  //Incase we ever decide to change representation in the future.
  typedef long long TimeStamp;
  typedef Eigen::Vector4d GroundTruth;
  typedef Eigen::VectorXd Measurement;

  enum SensorType{
    LASER,
    RADAR
  };

  struct SensorReading {
    TimeStamp timestamp;
    SensorType sensor_type;
    Measurement measurement;
  };

  //RMSE return type is a just a typedef for a Vector4d
  //RMSE stands for Root Squared Mean Error
  const Eigen::VectorXd CalculateRmse(const std::vector<Eigen::Vector4d> &estimations,
                           const std::vector<GroundTruth> &ground_truth);

  const Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  const Eigen::VectorXd PolarToCartesian(const Eigen::VectorXd& polar_vector);
  const Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& x_state);

  void CheckArguments(int argc, char* argv[]);
  void CheckFiles(std::ifstream& in_file, std::string& in_name,
                  std::ofstream& out_file, std::string& out_name);
};

#endif /* UTILITY_H_ */
