#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // set object covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;




}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

VectorXd FusionEKF::Polar2Cartesian(const MeasurementPackage &measurement_pack) {
      VectorXd result = VectorXd(4);
      double rho = measurement_pack.raw_measurements_[0]; // range
  	  double phi = measurement_pack.raw_measurements_[1]; // bearing
  	  double rho_dot = measurement_pack.raw_measurements_[2]; // velocity

  	  double x = rho * cos(phi);
      const double epsilon = 0.001;
      // Addressing division by zero issue
      if ( x < epsilon ) 
        x = epsilon;
      
  	  double y = rho * sin(phi);
      // Addressing division by zero issue
      if ( y < epsilon ) 
        y = epsilon;
      
  	  double vx = rho_dot * cos(phi);
  	  double vy = rho_dot * sin(phi);

      result << x, y, vx , vy;
      return result;
}


void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "EKF: radar " << endl;
      ekf_.x_ = Polar2Cartesian(measurement_pack);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "EKF: laser " << endl;
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_ ;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  // Elapsed time
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update the state transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Update the process noise covariance matrix
  // Noise value of 9 was value set in project description 
  const double noise_ax = 9;
  const double noise_ay = 9;

  // Calculate reusble parts
  double dt_2 = dt * dt; 
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt; 

  double dt_4_quarter = dt_4 / 4;
  double dt_3_half = dt_3 / 2;

  // Calculate process covariance matrix
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4_quarter * noise_ax,  0,                       dt_3_half * noise_ax, 0,
	           0,                        dt_4_quarter * noise_ay, 0,                    dt_3_half * noise_ay,
	           dt_3_half * noise_ax,     0,                       dt_2 * noise_ax,      0,
 	           0,                        dt_3_half * noise_ay,    0,                    dt_2 * noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //  Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
  	ekf_.R_ = R_radar_;
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
  	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
