#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "utility.h"
#include "fusion_ekf.h"

using namespace std;
using utility::SensorReading;
using utility::SensorType;
using utility::GroundTruth;
using utility::TimeStamp;
using utility::Measurement;
using utility::CheckArguments;
using utility::CheckFiles;
using utility::CalculateRmse;
using Eigen::VectorXd;

int main(int argc, char* argv[]) {

  CheckArguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  CheckFiles(in_file_, in_file_name_, out_file_, out_file_name_);

  vector<SensorReading> sensor_readings;
  vector<GroundTruth> ground_truths;

  string line;

  while (getline(in_file_, line)) {
    istringstream iss(line);

    string sensor_type;
    SensorReading sensor_reading;
    TimeStamp timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT
      float x,y;
      iss >> x; iss >> y; iss >> timestamp;

      sensor_reading.sensor_type = SensorType::LASER;
      sensor_reading.measurement = Measurement(2);
      sensor_reading.measurement << x, y;
      sensor_reading.timestamp = timestamp;
      sensor_readings.push_back(sensor_reading);
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT
      float ro, phi, ro_dot;
      iss >> ro; iss >> phi; iss >> ro_dot; iss >> timestamp;

      sensor_reading.sensor_type = SensorType::RADAR;
      sensor_reading.measurement = Measurement(3);
      sensor_reading.measurement << ro, phi, ro_dot;
      sensor_reading.timestamp = timestamp;
      sensor_readings.push_back(sensor_reading);
    }

    // read ground truth data to compare later
    float x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt; iss >> y_gt; iss >> vx_gt; iss >> vy_gt;

    GroundTruth ground_truth(4);
    ground_truth << x_gt, y_gt, vx_gt, vy_gt;
    ground_truths.push_back(ground_truth);
  }


  FusionEkf fusion_ekf_;
  vector<VectorXd> estimations;

  size_t N = sensor_readings.size();
  for (size_t k = 0; k < N; ++k) {
    SensorReading reading = sensor_readings[k];
    fusion_ekf_.ProcessMeasurement(reading);

    // output the estimation
    VectorXd x_ = fusion_ekf_.CurrentEstimate();

    out_file_ << x_(0) << "\t";
    out_file_ << x_(1) << "\t";
    out_file_ << x_(2) << "\t";
    out_file_ << x_(3) << "\t";


    if (reading.sensor_type == SensorType::LASER) {

      out_file_ << reading.measurement(0) << "\t";
      out_file_ << reading.measurement(1) << "\t";
    } else if (reading.sensor_type == SensorType::RADAR) {

      float ro = reading.measurement(0);
      float phi = reading.measurement(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    GroundTruth ground_truth = ground_truths[k];
    out_file_ << ground_truth(0) << "\t";
    out_file_ << ground_truth(1) << "\t";
    out_file_ << ground_truth(2) << "\t";
    out_file_ << ground_truth(3) << "\n";

    estimations.push_back(x_);
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl << CalculateRmse(estimations, ground_truths) << endl;


  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
