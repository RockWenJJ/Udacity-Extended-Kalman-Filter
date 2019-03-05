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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
   ekf_.x_ = VectorXd(4);
   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
			 0, 0, 1000, 0,
			 0, 0, 0, 1000;
			  
   ekf_.H_ = MatrixXd(2, 4);
   ekf_.H_ << 1, 0, 0, 0,
              0, 1, 0, 0;

   ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
   ekf_.Q_ = MatrixXd(4, 4);
   
}


/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_){
	  /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    // if the data comes from radar
	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR){
		// convert Radar data from polar to cartesian coordinates
		float rho = measurement_pack.raw_measurements_[0];
		float phi = measurement_pack.raw_measurements_[1];
		//float rho_dot = measurement_pack.raw_measurements_[2];
		
		ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;
	}
	// if the data comes from laser
	else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER){
		ekf_.x_ << measurement_pack.raw_measurements_[0],
		           measurement_pack.raw_measurements_[1],
				   0,
				   0;
	}
	previous_timestamp_ = measurement_pack.timestamp_;
	is_initialized_ = true;
    cout << "Initialized x: " << ekf_.x_ << endl;
	return;
  }
  
  float noise_ax = 8;
  float noise_ay = 8;
	
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  // transfer from milliseconds to seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt2*dt2;
  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
             0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
             dt3/2*noise_ax, 0, dt2*noise_ax, 0,
             0, dt3/2*noise_ay, 0, dt2*noise_ay;
  ekf_.Predict();
	
	
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	  cout << "Using Radar, x = " << ekf_.x_ <<endl;
    }
  else
      if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
	  cout << "Using Laser, x = " << ekf_.x_ << endl;
    }
}
