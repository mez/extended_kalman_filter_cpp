#include "utility.h"

using namespace std;

namespace utility {
  void CheckArguments(int argc, char* argv[]) {
    std::string usage_instructions = "Usage instructions: ";
    usage_instructions += argv[0];
    usage_instructions += " path/to/input.txt output.txt";

    bool has_valid_args = false;

    // make sure the user has provided input and output files
    if (argc == 1) {
      std::cerr << usage_instructions << std::endl;
    } else if (argc == 2) {
      std::cerr << "Please include an output file.\n" << usage_instructions << std::endl;
    } else if (argc == 3) {
      has_valid_args = true;
    } else if (argc > 3) {
      std::cerr << "Too many arguments.\n" << usage_instructions << std::endl;
    }

    if (!has_valid_args) {
      exit(EXIT_FAILURE);
    }
  }

  void CheckFiles(ifstream& in_file, string& in_name,
                   ofstream& out_file, string& out_name) {
    if (!in_file.is_open()) {
      std::cerr << "Cannot open input file: " << in_name << std::endl;
      exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) {
      std::cerr << "Cannot open output file: " << out_name << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  //  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  
  const RMSE CalculateRmse(const std::vector<Estimate> &estimations,
                     const std::vector<GroundTruth> &ground_truth) {
    RMSE rmse(4);
    rmse << 0,0,0,0;

    return rmse;
  }

};
