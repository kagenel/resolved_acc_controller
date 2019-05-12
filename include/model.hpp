#ifndef MODEL_HPP
#define MODEL_HPP

#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

class Model {
public:
  std::string name;
  int DoF;            // joint DoF or mass number
  int wDoF;           // work space DoF
  Eigen::VectorXd l;  // link length
  // Eigen::VectorXd x0; // initial state
  Eigen::VectorXd m;  // link mass
  Eigen::VectorXd I;  // moment of inertia of the link
  Eigen::VectorXd lc; // center of mass of the link

  Eigen::VectorXd rotation;
  
public:
  void read_model();
};

#endif // MODEL_HPP
