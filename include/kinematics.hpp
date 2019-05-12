#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <eigen3/Eigen/Dense>
#include "model.hpp"

class Kinematics {
public:
  Model model;
  
  Eigen::VectorXd position_ee;
  Eigen::MatrixXd jacobian;

  Eigen::VectorXd pos_des;
  Eigen::VectorXd vel_des;
  Eigen::VectorXd dth_ref; //reference joint velocity

  // Gain
  Eigen::MatrixXd Kp_v;

public:
  Eigen::VectorXd kinematics(double, Eigen::VectorXd);
  
  void read_kinematics();
  void calc_position_ee(Eigen::VectorXd);
  void calc_jacobian(Eigen::VectorXd);
  void calc_dth();

  void set_section_time(double);
  void set_desire(double);

  double polyrp(double, double, double, double, double, double);
  double polyrp(double, double, double, double, double, double, double, double);
  double dpolyrp(double, double, double, double, double, double, double, double);
  
};

#endif // KINEMATICS_HPP
