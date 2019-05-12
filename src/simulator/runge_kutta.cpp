#include <iostream>
#include "simulator.hpp"

Eigen::VectorXd Simulator::runge_kutta(double _time, Eigen::VectorXd _rotation){
  using namespace std;
  Eigen::MatrixXd k(cntl.model.DoF, 4);
  double runge_time;
  Eigen::VectorXd runge_rotation;
  
  // Phase 1
  k.col(0) = cntl.kinematics(_time, _rotation);
  
  // Phase 2
  runge_time = _time + sampling_time / 2.0;
  runge_rotation = _rotation + sampling_time * k.col(0) / 2.0;
  k.col(1) = cntl.kinematics(runge_time, runge_rotation);

  // Phase 3
  runge_time = _time + sampling_time / 2.0;
  runge_rotation = _rotation + sampling_time * k.col(1) / 2.0;
  k.col(2) = cntl.kinematics(runge_time, runge_rotation);
  
  // Phase 4
  runge_time = _time + sampling_time;
  runge_rotation = _rotation + sampling_time * k.col(2);
  k.col(3) = cntl.kinematics(runge_time, runge_rotation);
 
  // Last phase
  return _rotation + ((k.col(0) + 2 * k.col(1) + 2 * k.col(2) + k.col(3)) / 6.0) * sampling_time;
 
}
