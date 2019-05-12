#include "simulator.hpp"

void Simulator::forward_kinematics(Eigen::VectorXd _rotation) {
  cntl.calc_position_ee(_rotation);
}
