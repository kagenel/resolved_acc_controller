#include <iostream>
#include "kinematics.hpp"

void Kinematics::read_kinematics() {
  std::cout << "read_kinematics()" << std::endl;
  position_ee.resize(model.wDoF);
  jacobian.resize(model.wDoF, model.DoF);
  dth_ref.resize(model.DoF);

  pos_des.resize(model.wDoF);
  vel_des.resize(model.wDoF);
}
