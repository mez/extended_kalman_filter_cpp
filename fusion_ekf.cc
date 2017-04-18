#include "fusion_ekf.h"

using namespace utility;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

FusionEkf::FusionEkf() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

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

Eigen::VectorXd& FusionEkf::CurrentEstimate() {
  return ekf_.x_;
}
void FusionEkf::ProcessMeasurement(SensorReading& reading) {
  if (!is_initialized_) {
    //we will need to init the state vector x,y,vx,vy
    switch (reading.sensor_type) {
      case SensorType::LASER:
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << reading.measurement[0], reading.measurement[1], 0, 0;
        break;
      case SensorType::RADAR:
        ekf_.x_ = utility::PolarToCartesian(reading.measurement);
        break;
    }

    //we can't let x and y be zero.
    if ( fabs(ekf_.x_(0)+ekf_.x_(1)) < 1e-4){
      cout << "X and Y are practically Zero: "<< endl;
  		ekf_.x_(0) = 1e-4;
  		ekf_.x_(1) = 1e-4;
  	}

    previous_timestamp_ = reading.timestamp;
    is_initialized_ = true;
    return;
  }

  float dt = (reading.timestamp - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = reading.timestamp;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  //set the process covariance matrix Q
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  //Make Prediction
  ekf_.Predict();

  //Correct our prediction using the measurements
  if (reading.sensor_type == SensorType::LASER) {
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      VectorXd Hx = ekf_.H_ * ekf_.x_;

      ekf_.Update(reading.measurement, Hx);
  } else if (reading.sensor_type == SensorType::RADAR) {
      ekf_.H_ = CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
      VectorXd Hx = CartesianToPolar(ekf_.x_);

      ekf_.Update(reading.measurement, Hx);
  }
} //end FusionEkf::ProcessMeasurement
