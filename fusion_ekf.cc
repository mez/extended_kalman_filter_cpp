#include "fusion_ekf.h"

using namespace utility;
using Eigen::MatrixXd;
using Eigen::VectorXd;

FusionEkf::FusionEkf() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  float noise_ax = 9.0;
  float noise_ay = 9.0;

}

void FusionEkf::ProcessMeasurement(SensorReading& reading) {
  if (!is_initialized_) {
    //setup efk P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
			         0, 1, 0, 0,
			         0, 0, 1000, 0,
			         0, 0, 0, 1000;

    ekf_.x_ = VectorXd(4);

    //we will need to init the state vector x,y,vx,vy
    switch (reading.sensor_type) {
      case SensorType::LASER:
        break;
      case SensorType::RADAR:
        break;
    }




    return;
  }
} //end FusionEkf::ProcessMeasurement
