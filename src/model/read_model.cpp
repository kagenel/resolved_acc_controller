#include <iostream>
#include "model.hpp"

void Model::read_model() {
  std::cout << "read_model()" << std::endl;

  name = "Planar3R";
  DoF = 3;
  wDoF = 2;

  l.resize(DoF);
  l <<
    1., // [m]
    1., // [m]
    1.; // [m]

  m.resize(DoF);
  m <<
    1., // [kg]
    1., // [kg]
    1.; // [kg]

  
  I.resize(DoF);
  for(int i = 0; i < DoF; i++) {
      I(i) = m(i) * l(i) * l(i) / 12; // [kg m2]
  }

  lc = l / 2; // [m]

  rotation.resize(DoF);
  rotation <<
    30.0 * M_PI / 180.0,
    30.0 * M_PI / 180.0,
    30.0 * M_PI / 180.0;
}
