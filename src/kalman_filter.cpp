#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  std::cout << P_ << std::endl;
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  PartialUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double rho = sqrt(px*px + py*py);
  double theta = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;

  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot;

  VectorXd y = z - h;
  ClipToPi(y);
  PartialUpdate(y);
}

void KalmanFilter::PartialUpdate(const VectorXd &y){
  // Prepare identity matrix
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // Calculate new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::ClipToPi(VectorXd &v){
  while ( v(1) > M_PI || v(1) < -M_PI ) {
    if ( v(1) > M_PI ) {
      v(1) -= M_PI;
    } else {
      v(1) += M_PI;
    }
  }
}
