#include "kinematics.hpp"
#include <iostream>
void Kinematics::calc_dth() {
  dth_ref = (jacobian.transpose() * (jacobian * jacobian.transpose()).inverse()) * vel_des;
  
}
