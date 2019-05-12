#include "kinematics.hpp"

Eigen::VectorXd Kinematics::kinematics(double _time, Eigen::VectorXd _rotation) {

  //
  // Forward kinematics
  //
  calc_position_ee(_rotation);

  //
  // Polynomial interpolation
  //
  // set_desire(_time);
  
  //
  // Inverse kinematics
  //
  calc_jacobian(_rotation);
  calc_dth();

  return dth_ref;
}
