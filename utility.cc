#include "utility.h"

using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;

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

  const VectorXd CalculateRmse(const vector<VectorXd> &estimations,
                         const vector<VectorXd> &ground_truth) {
    VectorXd rmse;
    rmse << 0,0,0,0;
    if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
          cerr << "The estimation vector size should equal ground truth\
                        vector size. Also, the estimation vector size should\
                        not be zero" <<endl;
          return rmse;
    }

  	for(int i=0; i < estimations.size(); ++i){
  		VectorXd residual = estimations[i] - ground_truth[i];
  		residual = residual.array()*residual.array();

  		rmse += residual;
  	}

  	rmse /= estimations.size();
    rmse = rmse.array().sqrt();

    return rmse;
  }

  const MatrixXd CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    if (fabs(px) < 1e-4 and fabs(py) < 1e-4){
  	  px =  1e-4;
  	  py =  1e-4;
    }

    //pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    //check division by zero
    if(fabs(c1) < 1e-4){
      cerr << "CalculateJacobian () - Error - Division by Zero" << endl;
      c1 = 1e-6;
    }

    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
  }

  const VectorXd PolarToCartesian(const VectorXd& polar_vector) {
    VectorXd cart_vector(4);

    float rho = polar_vector(0);
    float phi = polar_vector(1);
    float rho_dot = polar_vector(2);

    cart_vector << rho * cos(phi), rho * sin(phi),
                   rho_dot * cos(phi), rho_dot * sin(phi);
    return cart_vector;
  }

  const Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& x_state) {
    VectorXd polar_vector(3);

    float x = x_state(0);
    float y = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float rho = sqrt(x*x + y*y);

    polar_vector << rho , atan2(y,x), (x*vx + y*vy)/rho;
    return polar_vector;
  }

};
