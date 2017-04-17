#ifndef UTILITY_H_
#define UTILITY_H_

#include <iostream>
#include <fstream>
#include <vector>
#include "libs/Eigen/Dense"
#include "measurement_packet.h"

namespace utility
{
  void CheckArguments(int argc, char* argv[]);
  void CheckFiles(std::ifstream& in_file, std::string& in_name,
                   std::ofstream& out_file, std::string& out_name);

  //RMSE return type is a just a typedef for a Vector4d
  //RMSE stands for Root Squared Mean Error
  const RMSE CalculateRmse(const std::vector<Estimate> &estimations,
                           const std::vector<GroundTruth> &ground_truth);
};

#endif /* UTILITY_H_ */
