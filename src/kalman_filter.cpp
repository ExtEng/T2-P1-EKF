#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
  x_ = F_* x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
  cout << "Predict: x_: "<< x_ << endl;
  cout << "Predict: P_: "<< x_ << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  // Predicted Measurement Vector
  VectorXd y_ = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
    
  // New Estimate
  x_ = x_ + K * y_;
  cout << "Udpate:x_: "<< x_ << endl;
  
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  cout << "Udpate:P_: "<< P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float x = x_(0);
  float y = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  cout << "UdpateEKF:x_ "<< x_<< endl;
  
  
  float rho = sqrt(x*x + y*y);
  float theta = atan2(y,x);
  
  cout << "UdpateEKF:Rho: "<< rho << endl;
  cout << "UdpateEKF:Theta: "<< theta << endl;
  
  float rho_dot;
  //check division by zero
  if(fabs(rho) < 0.0001){
	cout << "UdpateEKF:CalculationRho_dot: Error - Division By Zero" << endl;
	rho_dot = 0;
  } else {
	rho_dot = (x*vx + y*vy)/rho;
  }
    
  VectorXd zpred = VectorXd(3);
  zpred << rho, theta, rho_dot;
    
  VectorXd y_ = z - zpred;
  
  if( y_(1) > PI_ ) y_(1) -= 2*PI_;
  if( y_(1) < -PI_ ) y_(1) += 2*PI_;
  
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;
    
  // New Estimate
  x_ = x_ + K * y_;
  cout << "UdpateEKF: x_: "<< x_ << endl;
  
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  cout << "UdpateEKF: P_: "<< P_ << endl;
}
