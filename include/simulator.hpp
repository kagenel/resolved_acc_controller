#include <iostream>
#include <eigen3/Eigen/Dense>
#include "kinematics.hpp"

class Simulator {
public:
  // Simulation
  double time;
  double end_time;
  double section_time;
  double init_time;
  double sampling_time;
  double sampling_number;

  double gravity;

  Kinematics cntl;

  Eigen::VectorXd pos_start;
  Eigen::VectorXd vel_start;  
  
  Eigen::VectorXd pos_trgt;
  Eigen::VectorXd vel_trgt;  
  

public:
  void init();
  Eigen::VectorXd runge_kutta(double, Eigen::VectorXd);
  void set_command();
  void forward_kinematics(Eigen::VectorXd);
  void inverse_kinematics();

  void set_desire(double);

  double polyrp(double, double, double, double, double, double);
  double polyrp(double, double, double, double, double, double, double, double);
  double dpolyrp(double, double, double, double, double, double, double, double);
  
};
