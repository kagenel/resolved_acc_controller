#include <iostream>
#include "kinematics.hpp"

double Kinematics::polyrp (double t, double tf, double y0, double yf, double dy0, double dyf, double ddy0, double ddyf) {
 
  double a[6];
  double tf2 = tf*tf;
  double tf3 = tf*tf*tf;
  double tf4 = tf*tf*tf*tf; 
  double tf5 = tf*tf*tf*tf*tf;
  double par;

  a[0] = y0;
  a[1] = dy0;
  a[2] = ddy0/2.0;
  a[3] = (20.0 * yf - 20.0 * y0 - (8.0 * dyf + 12.0 * dy0) * tf - (3.0 * ddy0 -ddyf) * tf2) / (2.0 * tf3);
  a[4] = (30.0 * y0 - 30.0 * yf + (14.0 * dyf + 16.0 * dy0) * tf + (3.0 * ddy0 - 2.0 * ddyf) * tf2) / (2.0 * tf4);
  a[5] = (12.0 * yf - 12.0 * y0 - (6.0 * dyf + 6.0 * dy0) * tf - (ddy0 - ddyf) * tf2) / (2.0 * tf5);
  
  par = a[0] + a[1]*t + a[2]*t*t + a[3]*t*t*t + a[4]*t*t*t*t + a[5]*t*t*t*t*t;

  return par;
}

double Kinematics::dpolyrp (double t, double tf, double y0, double yf, double dy0, double dyf, double ddy0, double ddyf) {
 
  double a[6];
  double tf2 = tf*tf;
  double tf3 = tf*tf*tf;
  double tf4 = tf*tf*tf*tf; 
  double tf5 = tf*tf*tf*tf*tf;
  double par;

  a[0] = y0;
  a[1] = dy0;
  a[2] = ddy0/2.0;
  a[3] = (20.0 * yf - 20.0 * y0 - (8.0 * dyf + 12.0 * dy0) * tf - (3.0 * ddy0 -ddyf) * tf2) / (2.0 * tf3);
  a[4] = (30.0 * y0 - 30.0 * yf + (14.0 * dyf + 16.0 * dy0) * tf + (3.0 * ddy0 - 2.0 * ddyf) * tf2) / (2.0 * tf4);
  a[5] = (12.0 * yf - 12.0 * y0 - (6.0 * dyf + 6.0 * dy0) * tf - (ddy0 - ddyf) * tf2) / (2.0 * tf5);
  
  par = a[1] + 2.0*a[2]*t + 3.0*a[3]*t*t + 4.0*a[4]*t*t*t + 5.0*a[5]*t*t*t*t;

  return par;
}

double Kinematics::polyrp (double t, double tf, double y0, double yf, double dy0, double dyf) {
  
  double a[4];
  double tf2 = tf*tf;
  double tf3 = tf*tf*tf;
  double par;

  a[0] = y0;
  a[1] = dy0;
  a[2] = (3.0 * (yf - y0) - (2.0 * dy0 + dyf) * tf) / (tf2);
  a[3] = (-2.0 * (yf - y0) + (dyf + dy0) * tf) / (tf3);
  
  par = a[0] + a[1]*t + a[2]*t*t + a[3]*t*t*t;
  
  return par;
}

