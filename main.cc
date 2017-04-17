#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "libs/Eigen/Dense"
#include "measurement_packet.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[]) {

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  //Both SensorReading and GroundTruth come from measurement_packet.h
  vector<SensorReading> sensor_readings;
  vector<GroundTruth> ground_truths;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line)) {

    string sensor_type;
    SensorReading sensor_reading;
    GroundTruth ground_truth;
    istringstream iss(line);
    TimeStamp timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0) {
      // LASER MEASUREMENT
      sensor_reading.sensor_type = SensorType::LASER;
      sensor_reading.measurement = Measurement(2);
      float x,y;
      iss >> x;
      iss >> y;
      sensor_reading.measurement << x, y;
      iss >> timestamp;
      sensor_reading.timestamp = timestamp;
      sensor_readings.push_back(sensor_reading);
    } else if (sensor_type.compare("R") == 0) {
      // RADAR MEASUREMENT
      sensor_reading.sensor_type = SensorType::RADAR;
      sensor_reading.measurement = Measurement(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      sensor_reading.measurement << ro, phi, ro_dot;
      iss >> timestamp;
      sensor_reading.timestamp = timestamp;
      sensor_readings.push_back(sensor_reading);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    ground_truth = GroundTruth(4);
    ground_truth << x_gt, y_gt, vx_gt, vy_gt;
    ground_truths.push_back(ground_truth);
  }


  for(auto reading : sensor_readings) {
    cout << "Sensor Reading: "<< ((reading.sensor_type == LASER) ? "LASER":"RADAR") << "\nmeasurement: \n" << reading.measurement << endl;
  }

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
