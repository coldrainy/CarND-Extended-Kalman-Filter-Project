#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if(estimations.size()==0||estimations.size()!=ground_truth.size())
      return rmse;
  for(int i=0;i<estimations.size();i++){
      VectorXd residual = estimations[i]-ground_truth[i];
      residual = residual.array()*residual.array();
      rmse = rmse + residual;
  }
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  float x = x_state(0);
  float y = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
    MatrixXd Hj(3,4);
    float c1 = x*x+y*y;
    if(c1<0.000001){
      std::cout<<"error"<<std::endl;
      return Hj;
    }
    float c2 = sqrt(c1);
    float c3 = c2*c1;
    Hj << x/c2,y/c2,0,0,
          -y/c1,x/c1,0,0,
          y*(vx*y-vy*x)/c3,x*(vy*x-vx*y)/c3,x/c2,y/c2;
    return Hj;
}
