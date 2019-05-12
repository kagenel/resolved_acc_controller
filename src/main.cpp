#include <iostream>
#include <fstream>

#include "model.hpp"
#include "kinematics.hpp"
#include "simulator.hpp"

int main()
{
  using namespace std;
  
  Simulator sim;
  
  // ===================
  // Init
  // ===================
  sim.init();
  
  sim.cntl.model.name = "roll";
  
  cout << "model name: " << sim.cntl.model.name << endl;
  cout << "init rotation: " << sim.cntl.model.rotation.transpose() << endl;

  // -------------------
  // Set target
  // -------------------
  sim.pos_trgt(0) = 1.0;
  sim.pos_trgt(1) = 0.1;
  sim.vel_trgt(0) = 1.0;
  sim.vel_trgt(1) = 0.0;

  // ===================
  // 0 [s] Controller
  // ===================
  sim.cntl.kinematics(sim.init_time, sim.cntl.model.rotation);
  
  cout << "init pos (x,y): " << sim.cntl.position_ee.transpose() << endl;

  // ----------------------------
  // Set output
  // ----------------------------
  ofstream out_file("test.dat");  
  // int n = sim.sampling_number;

  // ===========
  // Start up
  // ===========
  sim.section_time = sim.end_time;
  sim.pos_start = sim.cntl.position_ee;
  Eigen::VectorXd sim_rotation = sim.cntl.model.rotation;
  
  for (int i=0;i<sim.sampling_number;i++) {
    // =============================
    // Update
    // =============================
    sim.time = sim.init_time + i * sim.sampling_time;
    sim.set_desire(sim.time);
    
    // =============================
    // Controller
    // =============================
    sim_rotation = sim.runge_kutta(sim.time, sim_rotation);
    
    // =============================
    // Output dat
    // =============================
    // out_file << sim.time << " " << sim.cntl.dth_ref.transpose() << endl;
    out_file << sim.time << " " << sim.cntl.position_ee.transpose() << endl;

  }

  out_file.close();

  return 0;
}
