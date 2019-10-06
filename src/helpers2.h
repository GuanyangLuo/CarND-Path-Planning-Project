#ifndef HELPERS2_H
#define HELPERS2_H

#include <iterator>
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"

using std::map;
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int getLane(double d, double lane_width){
  return (int) floor(d/lane_width);
}

double getD(int lane, double lane_width){
  return lane_width*lane + lane_width*0.5;
}

vector<double> fromCarToMapCoord(double car_x, double car_y, double car_theta, double car_i, double car_j){
  // Rotate  
  double rot_i = cos(car_theta) * car_i - sin(car_theta) * car_j;
  double rot_j = sin(car_theta) * car_i + cos(car_theta) * car_j;
  // Translate
  double map_x = rot_i + car_x;
  double map_y = rot_j + car_y;
  
  return {map_x, map_y};
}

vector<double> fromMapToCarCoord(double car_x, double car_y, double car_theta, double map_x, double map_y){
  // Translate
  double trans_x = map_x - car_x;
  double trans_y = map_y - car_y;
  // Rotate
  double car_i = cos(0-car_theta) * trans_x - sin(0-car_theta) * trans_y;
  double car_j = sin(0-car_theta) * trans_x + cos(0-car_theta) * trans_y;
  
  return {car_i, car_j};
}

vector<double> trajectoryStep(double s, double v, double a, double t){
  return {s + v*t + 0.5*a*t*t, 
          v + a*t, 
          a};
}

// vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
//   /**
//    * Calculate the Jerk Minimizing Trajectory that connects the initial state
//    * to the final state in time T.
//    *
//    * @param start - the vehicles start location given as a length three array
//    *   corresponding to initial values of [s, s_dot, s_double_dot]
//    * @param end - the desired end state for vehicle. Like "start" this is a
//    *   length three array.
//    * @param T - The duration, in seconds, over which this maneuver should occur.
//    *
//    * @output an array of length 6, each value corresponding to a coefficent in 
//    *   the polynomial:
//    *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
//    *
//    * EXAMPLE
//    *   > JMT([0, 10, 0], [10, 10, 0], 1)
//    *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
//    */

//   //Initialize the coefficents with a_0 = start_s, a_1 = s_dot, a_2 = 0.5 * s_double_dot
//   vector<double> coeff = {start[0], start[1], 0.5*start[2], 0.0, 0.0, 0.0}; 

//   //Solve the matrix form 
//   MatrixXd m(3,3);
//   m << pow(T,3), pow(T,4), pow(T,5),
//        3*pow(T,2), 4*pow(T,3), 5*pow(T,4),
//        6*T, 12*pow(T,2), 20*pow(T,3);

//   VectorXd rhs(3);
//   rhs << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T),
//          end[1] - (start[1] + start[2]*T),
//          end[2] - start[2];
         
//   VectorXd a_345 = m.colPivHouseholderQr().solve(rhs);
//   coeff[3] = a_345[0];
//   coeff[4] = a_345[1];
//   coeff[5] = a_345[2];
  
//   return coeff;
// }

// double trajectoryStepJMT(vector<double> coeff, double t){
//   return coeff[0] + coeff[1]*pow(t,1) + coeff[2]*pow(t,2) + coeff[3]*pow(t,3) + coeff[4]*pow(t,4) + coeff[5]*pow(t,5);
// }

#endif  // HELPERS2_H