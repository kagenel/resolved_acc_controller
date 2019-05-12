#include <iostream>
#include "simulator.hpp"

void Simulator::init() {
  cntl.model.read_model();
  cntl.read_kinematics();

  time = 0.0;
  init_time = 0.0;
  end_time = 10.0;
  sampling_time = 1.0e-3;
  sampling_number = (end_time - init_time) / sampling_time;

  gravity = 9.81665; // [m/s2]

  pos_start.resize(cntl.model.wDoF);
  vel_start.resize(cntl.model.wDoF);
  pos_trgt.resize(cntl.model.wDoF);
  vel_trgt.resize(cntl.model.wDoF);
}
