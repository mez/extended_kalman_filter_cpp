#ifndef MEASUREMENT_PACKET_H_
#define MEASUREMENT_PACKET_H_

#include "libs/Eigen/Dense"

typedef long long TimeStamp;
typedef Eigen::VectorXd GroundTruth;
typedef Eigen::VectorXd Measurement;

enum SensorType{
  LASER,
  RADAR
};

//captures the sensor reading here.
struct SensorReading {
  TimeStamp timestamp;
  SensorType sensor_type;
  Measurement measurement;
  GroundTruth ground_truth;
};


#endif /* MEASUREMENT_PACKET_H_ */
