#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residue = estimations[i] - ground_truth[i];
    residue = residue.array()*residue.array();
    rmse += residue;
  }

  // calculate the mean
  rmse = rmse / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);
   
   float pxy_2 = px * px + py * py;
   float pxy_sqrt = sqrt(pxy_2);
   float pxy_32 = pxy_2 * pxy_sqrt;
   
   MatrixXd Hj(3, 4);
   // in case pxy_2 is 0
   if(pxy_2 < 1e-8){
	   Hj << 0, 0, 0, 0,
	         0, 0, 0, 0,
			 0, 0, 0, 0;
	   std::cout << "Function CalculateJacobian() has Error: Division by Zero" << std::endl;
	   return Hj;
   }
   Hj << px/pxy_sqrt, py/pxy_sqrt, 0, 0,
         -py/pxy_2, px/pxy_2, 0, 0,
		 py*(vx*py - vy*px)/pxy_32, px*(vy*px - vx*py)/pxy_32, px/pxy_sqrt, py/pxy_sqrt;
   return Hj;
}
