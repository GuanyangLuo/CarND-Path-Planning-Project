#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include <cmath>

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
  /**
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions used:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   */
  
    //only consider states which can be reached from current FSM state.
    vector<string> possible_successor_states =  successor_states();
    
    //keep track of the total cost of each state.
    std::map<string, float> costs;
    std::map<string, vector<Vehicle>> trajectories;
    for (auto state : possible_successor_states){
        //generate a rough idea of what trajectory we would
        //follow IF we chose this state.
        vector<Vehicle> trajectory_for_state = generate_trajectory(state, predictions);
    
        //calculate the "cost" associated with that trajectory.
        float cost_for_state = calculate_cost(*this, predictions, trajectory_for_state);
        costs[state] = cost_for_state;
        trajectories[state] = trajectory_for_state;
    }
    
    //Find the minimum cost state.
    string best_next_state;
    float min_cost = 9999999;
    for (auto state : possible_successor_states){
        float cost = costs[state];
        if (cost < min_cost){
            min_cost = cost;
            best_next_state = state;
        }
    }   
    
  return trajectories[best_next_state];
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  
  // TODO: Implement switching across two lanes
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
//     states.push_back("PLCL2");
//     states.push_back("PLCR2");
  } else if (state.compare("PLCL") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }    
  } else if (state.compare("LCL") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("LCL");
    }
  } else if (state.compare("LCR") == 0) {
    if (lane != 0) {
      states.push_back("LCR");
    }
//   } else if (state.compare("PLCL2") == 0) {
//     if (lane == 0) {
//       states.push_back("PLCL2");
//       states.push_back("LCL2");
//     }
//   } else if (state.compare("PLCR2") == 0) {
//     if (lane == lanes_available - 1) {
//       states.push_back("PLCR2");
//       states.push_back("LCR2");
//     }    
//   } else if (state.compare("LCL2") == 0) {
//     if (lane == 0) {
//       states.push_back("LCL2");
//     }
//   } else if (state.compare("LCR2") == 0) {
//     if (lane == lanes_available - 1) {
//       states.push_back("LCR2");
//     }
  }  
  
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, 
                                             map<int, vector<Vehicle>> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
//   } else if (state.compare("LCL2") == 0 || state.compare("LCR2") == 0) {
//     trajectory = lane_change_trajectory(state, predictions);
//   } else if (state.compare("PLCL2") == 0 || state.compare("PLCR2") == 0) {
//     trajectory = prep_lane_change_trajectory(state, predictions);
  }

  return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions, 
                                      int lane) {
  // Gets next timestep kinematics (position, velocity, acceleration) 
  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
  //   given other vehicle positions and accel/velocity constraints.
  
  float t = this->planning_time;
  float max_velocity_accel_limit = this->max_acceleration * t + this->v;
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    this->ahead_dist = vehicle_ahead.s - this->s;
    this->ahead_v = vehicle_ahead.v;

    // Solving the equation: (ahead.s + ahead.v*t) - (this.s + this.v*t + 0.5*a_max*t*t) = buffer
    float t_buffer = 1.0; // Give 1 second to reach the buffer
    float max_accel_in_front = (
                                  (vehicle_ahead.s + vehicle_ahead.v * t_buffer)
                                  - (this->s + this->v * t_buffer)
                                  - this->preferred_buffer
                                )
                                  / (0.5 * t_buffer * t_buffer);
    float max_velocity_in_front = this->v + max_accel_in_front * t;
    new_velocity = std::min(std::min(max_velocity_in_front, 
                                     max_velocity_accel_limit), 
                            this->target_speed);
  } else {
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
  }

  new_accel = (new_velocity - this->v)/t; // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + this->v * t + new_accel / 2.0 * t * t;
  
  return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(this->planning_time);
  vector<Vehicle> trajectory = {Vehicle(this->lane,this->s,this->v,this->a,this->state), 
                                Vehicle(this->lane,next_pos,this->v,0,this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
  vector<float> kinematics = get_kinematics(predictions, this->lane);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
  
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, 
                                                     map<int, vector<Vehicle>> &predictions) {
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, 
                                        this->state)};
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];    
  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
  
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, 
                                                map<int, vector<Vehicle>> &predictions) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies 
  //   that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (fabs(next_lane_vehicle.s - this->s) < this->preferred_buffer && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return to previous state.
      if (state.compare("LCL") == 0)
        trajectory = prep_lane_change_trajectory("PLCL", predictions);
      else if (state.compare("LCR") == 0)
        trajectory = prep_lane_change_trajectory("PLCR", predictions);
//       else if (state.compare("LCL2") == 0)
//         trajectory = prep_lane_change_trajectory("PLCL2", predictions);
//       else if (state.compare("LCR2") == 0)
//         trajectory = prep_lane_change_trajectory("PLCR2", predictions);
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a, 
                               this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], 
                               kinematics[2], state));
  return trajectory;
}

float Vehicle::position_at(int t) {
  return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions, 
                                 int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  float max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[this->prediction_index]; // Get the predicted vehicle state at a specific time
    if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s 
        && temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, 
                                int lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  float min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[this->prediction_index]; // Get the predicted vehicle state at a specific time
    if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s 
        && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(float horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory 
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  
  float time_step = this->time_step;
  int horizon_iteration = (int) horizon/time_step;
  for(int i = 0; i < horizon_iteration; ++i) {
    float next_s = position_at(i*time_step);
//     float next_v = 0;
//     if (i < horizon_iteration-1) {
//       next_v = position_at((i+1)*time_step) - s;
//     }
    float next_v = this->v;
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }
  
  return predictions;
}

// void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
//   // Sets state and kinematics for ego vehicle using the last state of the trajectory.
//   Vehicle next_state = trajectory[1];
//   this->state = next_state.state;
//   this->lane = next_state.lane;
//   this->s = next_state.s;
//   this->v = next_state.v;
//   this->a = next_state.a;
// }

// void Vehicle::configure(vector<int> &road_data) {
//   // Called by simulator before simulation begins. Sets various parameters which
//   //   will impact the ego vehicle.
//   target_speed = road_data[0];
//   lanes_available = road_data[1];
//   goal_s = road_data[2];
//   goal_lane = road_data[3];
//   max_acceleration = road_data[4];
// }