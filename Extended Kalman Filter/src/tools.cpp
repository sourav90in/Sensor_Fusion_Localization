#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
   VectorXd rmse(4);
   rmse << 0,0,0,0;

   // check the validity of the following inputs:
   //  * the estimation vector size should not be zero
   //  * the estimation vector size should equal ground truth vector size
   if(estimations.size() != ground_truth.size()
	    || estimations.size() == 0){
	cout << "Invalid estimation or ground_truth data" << endl;
	return rmse;
   }

   //accumulate squared residuals
   for(unsigned int i=0; i < estimations.size(); ++i){
       VectorXd residual = estimations[i] - ground_truth[i];
       //coefficient-wise multiplication
       residual = residual.array()*residual.array();
       rmse += residual;
   }

   //calculate the mean
   rmse = rmse/estimations.size();

   //calculate the squared root
   rmse = rmse.array().sqrt();

   //return the result
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);

  Hj<< 0,0,0,0,
	   0,0,0,0,
	   0,0,0,0;

  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float pxy = px*px + py*py;

  if(pxy == 0.0) return Hj;

  Hj(0,0) = px/sqrt(pxy);
  Hj(0,1) = py/sqrt(pxy);
  Hj(1,0) = -py/pxy;
  Hj(1,1) = px/pxy;
  Hj(2,0) = py*(vx*py - vy*px)/(sqrt(pxy*pxy*pxy));
  Hj(2,1) = px*(vy*px - vx*py)/(sqrt(pxy*pxy*pxy));
  Hj(2,2) = px/sqrt(pxy);
  Hj(2,3) = py/sqrt(pxy);

  return Hj;

}
