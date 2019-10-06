#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "helpers2.h"
#include "vehicle.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  // FSM global variable
  string FSM_STATE = "KL";
  int TIMER = 0;
  // Logging
  std::ofstream LOG_FILE;
  bool GENERATE_LOG = true;
  if (GENERATE_LOG)
    LOG_FILE.open ("log.txt");
  
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy
              ,&FSM_STATE, &TIMER, &GENERATE_LOG, &LOG_FILE // new variables
              ]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * Define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Parameters
          const double LANE_WIDTH = 4.0; // meter
          const int NUM_LANE = 3;
          const double MAX_ACCELERATION = 5; // meter/second^2 (less than 10 so as to maintain some buffer)
          const double TARGET_SPEED = 45 / 2.24; // meter/second (less than 50 MPH so as to maintain some buffer)
          const double SENSOR_RANGE_FRONT = 75; // meter (only check cars in this range)
          const double SENSOR_RANGE_BACK = -15; // meter (only check cars in this range)
          const int PREFERRED_BUFFER = 15; // meter (buffer from the car ahead of and behind ego)
          
          const double TIME_STEP = 0.02; // second
          const double PLANNING_TIME = 1.5; // second
          const int STEPS_TO_PLAN = (int) PLANNING_TIME / TIME_STEP;          
          const int PREV_KEEP = 5; // Number of data to keep from previous trajectory
          const int MIN_PREV_SIZE = PREV_KEEP + 3; // Need 3 additional data point to calculate a state for starting new trajectory
          const int START_INDEX = PREV_KEEP - 1; // Index (in previous path) of the starting point for new trajectory
          
          const int FSM_STATE_UPDATE_ITERATION = 5; // Update the state of the FSM every 5 time_step so that it doesn't abruptly change trajectory 
          TIMER++;
          
          // Logging
          if (GENERATE_LOG && !(LOG_FILE.is_open())){
            std::cout << "log file not opened" << std::endl;
            GENERATE_LOG = false;
          }
          
          
          // Plan path as a continuation of the previous path in order to handle lantency
          int prev_size = previous_path_x.size();

          std::cout <<"car: "<<" s: "<<car_s<<" d: "<< car_d <<" v(MPH): "<< car_speed <<" prev_size: "<<prev_size<<" logging: "<<GENERATE_LOG<< std::endl;
          if (GENERATE_LOG)
            LOG_FILE <<"car: "<<" s: "<<car_s<<" d: "<< car_d <<" v(MPH): "<< car_speed <<" prev_size: "<<prev_size<< std::endl;

          double start_s, start_d, start_speed, start_accel, start_theta;
          vector<double> start_xy;
          int prediction_index;
          if (prev_size < MIN_PREV_SIZE){ // First time planning path
            prediction_index = 0;
            start_s = car_s;
            start_d = car_d;
            start_speed = car_speed / 2.24; // Convert from MPH to meter
            start_accel = 0;
            start_theta = deg2rad(car_yaw);
            start_xy = {car_x, car_y};
          }else{ // Has enough data to continue from the previous path
            prediction_index = START_INDEX;
            // Copy some data from the beginning of the previous path to next_x(y)_vals
            for (int i = 0; i < PREV_KEEP; i++){
              double px = previous_path_x[i];
              double py = previous_path_y[i];
              next_x_vals.push_back(px);
              next_y_vals.push_back(py);
              
              if (GENERATE_LOG){
                double px2 = previous_path_x[i+1];
                double dx = px2 - px;
                double py2 = previous_path_y[i+1];
                double dy = py2 - py;
                vector<double> old_sd = getFrenet(px,py,atan2(dy,dx), map_waypoints_x, map_waypoints_y);
                LOG_FILE << "old: " << " x: " << px << " y: " << py << " s: " << old_sd[0] << " d: " << old_sd[1] << " v: " << sqrt(dx*dx+dy*dy)/TIME_STEP << std::endl;
              } 

            }
            
            // Use 3 additional data point to calculate a state for starting new trajectory
            vector<double> state_theta, state_speed;
            for (int i = START_INDEX; i < START_INDEX + 2; i++){
              double x2 = previous_path_x[i+1];
              double x1 = previous_path_x[i];
              double y2 = previous_path_y[i+1];
              double y1 = previous_path_y[i];
              double dx = x2 - x1;
              double dy = y2 - y1;
              state_theta.push_back(atan2(dy,dx));
              state_speed.push_back(sqrt(dy*dy+dx*dx)/TIME_STEP);
            }
            
            start_theta = state_theta[0];
            vector<double> start_sd = getFrenet(previous_path_x[START_INDEX],previous_path_y[START_INDEX],start_theta, map_waypoints_x, map_waypoints_y);
            start_s = start_sd[0];
            start_d = start_sd[1];
            start_speed = state_speed[0];
            start_accel = (state_speed[1]-state_speed[0])/TIME_STEP;
            start_xy = {previous_path_x[START_INDEX], previous_path_y[START_INDEX]};
          }
          
          
          // Prediction
          map<int ,vector<Vehicle>> predictions;
          for (auto it : sensor_fusion) {
            // Format for sensor_fusion is [id, x, y, vx, vy, s, d]
            float s = (float) it[5];
            if ((s - car_s) <= SENSOR_RANGE_FRONT && (s - car_s) >= SENSOR_RANGE_BACK){    
              // Setup Vehicle based on sensor_fusion data
              int v_id = (int) it[0];
              float vx = (float) it[3];
              float vy = (float) it[4];
              float speed = (float) sqrt(vx*vx + vy*vy);
              int lane = getLane(it[6], LANE_WIDTH);
              Vehicle v(lane, s, speed, 0, "CS");
              v.time_step = TIME_STEP;
              v.planning_time = PLANNING_TIME;

              // Generate Predictions
              float horizon = (float) PLANNING_TIME;
              vector<Vehicle> preds = v.generate_predictions(horizon);
              predictions[v_id] = preds;
            }
          }
          
          
          // Finite State Machine (Based on the "Behavior Planning" lesson)
          int lane = getLane(start_d, LANE_WIDTH);
          Vehicle ego(lane, start_s, start_speed, start_accel);          
          ego.state = FSM_STATE; 
          ego.target_speed = TARGET_SPEED;
          ego.lanes_available = NUM_LANE;
          ego.goal_s = start_s + 6945.554;// Keep driving
          ego.goal_lane = 1; // Default to the center lane so than it is easier to maneuver around traffic  
          ego.max_acceleration = MAX_ACCELERATION;
          ego.preferred_buffer = PREFERRED_BUFFER;
          ego.time_step = TIME_STEP;
          ego.planning_time = PLANNING_TIME;
          ego.prediction_index = prediction_index;
          if (GENERATE_LOG)
            LOG_FILE <<"FSM_start: "<<" state: "<<ego.state<<" s: "<<ego.s<<" lane: "<<ego.lane<<" v: "<<ego.v<<" a: "<<ego.a<<" th: "<<start_theta<<" ahead_dist: "<<ego.ahead_dist<<" ahead_v: "<<ego.ahead_v<< std::endl;
          
          
          // Use the FSM to plan the best trajectory
          // TODO: Can be improved by making lane change happens only when ego is close to the traffic ahead,
          //       so that it sees more of the traffic and can plan better
          vector<Vehicle> trajectory_endpoints = ego.choose_next_state(predictions);
          Vehicle ego_end = trajectory_endpoints[1];
          double end_s = ego_end.s;
          double end_d = getD(ego_end.lane, LANE_WIDTH);
          if (TIMER%FSM_STATE_UPDATE_ITERATION == 0)
            FSM_STATE = ego_end.state;  
          if (GENERATE_LOG)
            LOG_FILE <<"FSM_end: "<<" state: "<<ego_end.state<<" s: "<<ego_end.s<<" lane: "<<ego_end.lane<<" v: "<<ego_end.v<<" a: "<<ego_end.a<< std::endl;
      
          
//           // JMT
//           vector<double> s_state_start = {ego.s,ego.v,ego.a}, s_state_end = {ego_end.s,ego_end.v,ego_end.a};
//           vector<double> d_state_start = {start_d,0,0}, d_state_end = {end_d,0,0};
//           vector<double> s_coeff = JMT(s_state_start, s_state_end, PLANNING_TIME);
//           vector<double> d_coeff = JMT(d_state_start, d_state_end, PLANNING_TIME);
          
          
          // Generate spline for the trajectory
          // The shape of the spline will change at each time step
          vector<double> end_xy = getXY(end_s, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> spline_control_end = getXY(end_s+30, end_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<vector<double>> spline_points_xy;
          if (prev_size < MIN_PREV_SIZE){ // First time planning path
            spline_points_xy = {start_xy, end_xy, spline_control_end};
          } else { // Has enough data to continue from the previous path
            vector<double> spline_control_start = {previous_path_x[START_INDEX - 1], previous_path_y[START_INDEX - 1]};
            spline_points_xy = {spline_control_start, start_xy, end_xy, spline_control_end};
          }
          
          // Spline points should be in the car coordinates to prevent the spline becoming a multi-valued function
          vector<vector<double>> spline_points = {{},{}};
          for (int i = 0; i < spline_points_xy.size(); i++){
            double point_x = spline_points_xy[i][0];
            double point_y = spline_points_xy[i][1];
            vector<double> point_ij = fromMapToCarCoord(start_xy[0], start_xy[1], start_theta, point_x, point_y);
            spline_points[0].push_back(point_ij[0]);
            spline_points[1].push_back(point_ij[1]);
            if (GENERATE_LOG)
              LOG_FILE <<"point_x: "<<point_x<<" point_y: "<<point_y<<" point_i: "<<point_ij[0]<<" point_j: "<<point_ij[1]<< std::endl;
          }
          tk::spline trajectory_spline;
          trajectory_spline.set_points(spline_points[0],spline_points[1]);
          
          
          // Trajectory
          double step_s = ego.s, step_v = ego.v, step_a = ego_end.a, step_d = 0;  // The acceleration at the end is the same  
          double end_i = end_s - start_s, end_j = trajectory_spline(end_i);
          double linear = end_i / sqrt(end_i*end_i + end_j*end_j); // A factor to linearize steps along the spline
          for (int step = 1; step < STEPS_TO_PLAN; step++){ // Skip start_xy that is already kept from previous path
//             step_s = trajectoryStepJMT(s_coeff, step*TIME_STEP);
//             step_d = trajectoryStepJMT(d_coeff, step*TIME_STEP);
//             vector<double> trajectory_xy = getXY(step_s, step_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            vector<double> sva = trajectoryStep(step_s, step_v, step_a, TIME_STEP); //one time step
            step_s = sva[0], step_v = sva[1], step_a = sva[2];
            double trajectory_i = (step_s - start_s) * linear; // From Frenet coordinates to car coordinates
            double trajectory_j = trajectory_spline(trajectory_i);
            // Convert a point on the spline from the car's coordinates to the map's coordinates
            vector<double> trajectory_xy = fromCarToMapCoord(start_xy[0], start_xy[1], start_theta, trajectory_i, trajectory_j);
            
            next_x_vals.push_back(trajectory_xy[0]);
            next_y_vals.push_back(trajectory_xy[1]);
            
            if (GENERATE_LOG)
              LOG_FILE <<"new: "<<" x: "<<trajectory_xy[0]<<" y: "<<trajectory_xy[1]<<" i: "<<trajectory_i<<" j: "<<trajectory_j<<" s: "<<step_s<<" v: "<<step_v<<" a: "<<step_a<< std::endl;
          }
          
          
          if (GENERATE_LOG)
            LOG_FILE << "path size: "<<next_x_vals.size() << std::endl;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}