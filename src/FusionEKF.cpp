#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_<<1,0,0,0,
            0,1,0,0;
  MatrixXd P(4,4);
  P<<1,0,0,0,
     0,1,0,0,
     0,0,1,0,
     0,0,0,1;
  ekf_.P_ = P;

  MatrixXd F(4,4);
  F<<1,0,0,0,
     0,1,0,0,
     0,0,1,0,
     0,0,0,1;
  ekf_.F_ = F;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    MatrixXd P(4,4);
    P<<1,0,0,0,
     0,1,0,0,
     0,0,1000,0,
     0,0,0,1000;
    ekf_.P_ = P;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

        float ro = measurement_pack.raw_measurements_(0);
        float theta = measurement_pack.raw_measurements_(1);
        float ro_dot = measurement_pack.raw_measurements_(2);
        float x = ro*cos(theta);
        float y = ro*sin(theta);
        float vx = ro_dot*cos(theta);
        //float vx = 0;
        float vy = ro_dot*sin(theta);
        //float vy = 0;
        ekf_.x_(0) = x;
        ekf_.x_(1) = y;
        ekf_.x_(2) = vx;
        ekf_.x_(3) = vy;
        ekf_.R_ = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */

        float x = measurement_pack.raw_measurements_(0);
        float y = measurement_pack.raw_measurements_(1);
        ekf_.x_(0) = x;
        ekf_.x_(1) = y;
        ekf_.x_(2) = 0;
        ekf_.x_(3) = 0;
        ekf_.R_ = R_laser_;
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  long long current_time = measurement_pack.timestamp_;
  float dt = (current_time - previous_timestamp_)/1000000.0;
  previous_timestamp_ = current_time;
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;
  float noise_ax = 9;
  float noise_ay = 9;
  float dt_2 = dt*dt;
  float dt_3 = dt*dt_2;
  float dt_4 = dt*dt_3;
  MatrixXd Q(4,4);
  Q<<dt_4/4*noise_ax,0,dt_3/2*noise_ax,0,
     0,dt_4/4*noise_ay,0,dt_3/2*noise_ay,
     dt_3/2*noise_ax,0,dt_2*noise_ax,0,
     0,dt_3/2*noise_ay,0,dt_2*noise_ay;
  ekf_.Q_ = Q;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
