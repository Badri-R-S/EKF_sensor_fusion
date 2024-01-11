#include "kalman_filter.h"

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
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_; // State Transition Function 
  MatrixXd Ft = F_.transpose(); // Transpose to Multiply 
  P_ = F_ * P_ * Ft + Q_; // State covariance Matrix 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_; // Measurement prediction
  VectorXd y = z - z_pred; // Measurement Error 
  MatrixXd Ht = H_.transpose(); // Measurement Function Transpose 
  MatrixXd S = H_ * P_ * Ht + R_; // Measurement Prediction Covariance 
  MatrixXd Si = S.inverse(); // Inverse to Multiply 
  MatrixXd PHt = P_ * Ht; // Break out Multiplication 
  MatrixXd K = PHt * Si; // Kalman Gain 

  // new state
  x_ = x_ + (K * y); // Best Estimate 
  long x_size = x_.size(); // size modifier 
  MatrixXd I = MatrixXd::Identity(x_size, x_size); // Identity matrix, used for matrix inversion 
  P_ = (I - K * H_) * P_; // Uncertainty Covariance 
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  //readings convert from cartesian coordinates to polar coordinates.
  float rho = sqrt((px * px) + (py * py));                                                                  
  float phi = atan2(py, px); 
  float rho_dot; 
  
  // check if rho is zero	
  if (fabs(rho) < 0.0001) 
  {
    	rho_dot = 0;
  } 
  else 
  {
    rho_dot = (px*vx + py*vy)/rho;
  }
  
  
  VectorXd z_pred(3); /* measurement vector 3 parameters */
  z_pred << rho, 
            phi, 
            rho_dot;
  
  VectorXd y = z - z_pred; /* Measurement Error */

  //phi should be between -pi and pi
  if (y(1) < -M_PI) // y(1) refers to phi
    {
      y(1) += 2 * M_PI;
    }
    
    else if (y(1) > M_PI)
    {
      y(1) -= 2 * M_PI;
    }
  
  MatrixXd Ht = H_.transpose(); //Measurement Function 
  MatrixXd S = H_ * P_ * Ht + R_; // Measurement Prediction Covariance
  MatrixXd Si = S.inverse(); // Inverse to Multiply 
  MatrixXd PHt = P_ * Ht; 
  MatrixXd K = PHt * Si; // Kalman Gain

  //new estimate
  x_ = x_ + (K * y); // Best Estimate 
  long x_size = x_.size(); // size modifier
  MatrixXd I = MatrixXd::Identity(x_size, x_size); // Identity matrix, used for matrix inversion 
  P_ = (I - K * H_) * P_; // Uncertainty Covariance 
}
