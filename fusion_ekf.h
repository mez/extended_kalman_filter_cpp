#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_
//This EKF fusion handles a CV model hence the state vector will be 4D.
// [px, py, vx, vy] we denote this by the typedef utility::Estimate

#include <iostream>
#include "libs/Eigen/Dense"
#include "ekf.h"
#include "utility.h"

class FusionEkf {

 public:
  FusionEkf();
  void ProcessMeasurement(utility::SensorReading& reading);
  Eigen::VectorXd current_estimate();

 private:
  bool is_initialized_;
  long long previous_timestamp_;

  //This Fusion class handles LIDAR and RADAR
  //So we set the sensor measurement maxtrices and covar for the sensors
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd R_radar_;

  //since this is a constant velocity model. we add the accel as noise.
  float noise_ax_;
  float noise_ay_;

  /**
  * Extended Kalman Filter update and prediction math lives in here.
  */
  ExtendedKalmanFilter ekf_;

  void UpdateFQ(float dt);
};

#endif /* FUSION_EKF_H_ */
