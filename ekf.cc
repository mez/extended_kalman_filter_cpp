#include "ekf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ExtendedKalmanFilter::ExtendedKalmanFilter() {
  //setup efk F
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  //setup efk P
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
}

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
