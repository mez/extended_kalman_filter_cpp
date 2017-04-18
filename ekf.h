#ifndef EKF_H_
#define EKF_H_

#include "libs/Eigen/Dense"
#include "utility.h"

class ExtendedKalmanFilter {
 public:
   // state vector
   Eigen::VectorXd x_;

   // state covariance matrix
   Eigen::MatrixXd P_;

   // state transistion matrix
   Eigen::MatrixXd F_;

   // process covariance matrix
   Eigen::MatrixXd Q_;

   // measurement matrix
   Eigen::MatrixXd H_;

   // measurement covariance matrix
   Eigen::MatrixXd R_;

   //4x4 Identity matrix we will need later.
   Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(4, 4);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z, const Eigen::VectorXd& Hx);
};

#endif
