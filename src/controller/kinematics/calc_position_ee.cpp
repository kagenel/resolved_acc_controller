#include "kinematics.hpp"
#include <iostream>

void Kinematics::calc_position_ee(Eigen::VectorXd _rotation) {
  using namespace std;
  
  double s1   = sin(_rotation(0));
  double c1   = cos(_rotation(0));
  double s12  = sin(_rotation(0) + _rotation(1));
  double c12  = cos(_rotation(0) + _rotation(1));
  double s123 = sin(_rotation(0) + _rotation(1) + _rotation(2));
  double c123 = cos(_rotation(0) + _rotation(1) + _rotation(2));

  position_ee <<
    model.l(0) * c1 + model.l(1) * c12 + model.l(2) * c123,
    model.l(0) * s1 + model.l(1) * s12 + model.l(2) * s123;
}
