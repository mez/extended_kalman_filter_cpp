#include "ekf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using utility::CartesianToPolar;

void ExtendedKalmanFilter::Predict() {
  x_ = F_ * x_ ;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_; // error calculation
  CallRestOfUpdate(y);
}

void ExtendedKalmanFilter::UpdateEkf(const VectorXd &z) {
  VectorXd hx = CartesianToPolar(x_);
  VectorXd y = z - hx;
  CallRestOfUpdate(y);
}

void ExtendedKalmanFilter::CallRestOfUpdate(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K =  P_ * Ht * S.inverse();

  // New state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}
