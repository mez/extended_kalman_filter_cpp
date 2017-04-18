#include "ekf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void ExtendedKalmanFilter::Predict() {
  x_ = F_ * x_ ;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::Update(const VectorXd &z, const VectorXd& Hx) {
  VectorXd y = z - Hx; // error calculation

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd K =  P_ * Ht * S.inverse();

  // New state
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}
