#include "simulator.hpp"

void Simulator::set_desire(double _time) {
  cntl.pos_des(0) =  polyrp(_time, section_time, pos_start(0), pos_trgt(0), 0, 0, 0, 0);
  cntl.pos_des(1) =  polyrp(_time, section_time, pos_start(1), pos_trgt(1), 0, 0, 0, 0);
  cntl.vel_des(0) = dpolyrp(_time, section_time, pos_start(0), pos_trgt(0), 0, 0, 0, 0);
  cntl.vel_des(1) = dpolyrp(_time, section_time, pos_start(1), pos_trgt(1), 0, 0, 0, 0);
}
