#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
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
  
  // start in lane 1 where the left-most lane is 0
  int lane = 1;
  
  // Have a reference velocity in mph to target
  double ref_vel = 0.0; //mph
  
  // Current state
  string curr_state = "KL";

  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &curr_state]
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
          
          int prev_size = previous_path_x.size();
          
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          // FSM and cost function
          // Populate possible states
          vector<string> states;
          states.push_back("KL");
          if(curr_state.compare("KL") == 0) {
            states.push_back("PLCL");
            states.push_back("PLCR");
          } else if (curr_state.compare("PLCL") == 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
          } else if (curr_state.compare("PLCR") == 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
          } else if (curr_state.compare("LCL") == 0) {
            states.push_back("LCL");
            states.push_back("KL");
          } else if (curr_state.compare("LCR") == 0) {
            states.push_back("LCR");
            states.push_back("KL");
          }
          
          // Loop over cars detected by sensor fusion
          //  find ref_v to use
          bool too_close = false;
          int best_cost = 100;
          int temp_cost = 0;
          int best_idx = 0;
          double VEHICLE_RADIUS = 2.0;
          for (int i=0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            double check_car_s = sensor_fusion[i][5];
            // if using previous points can project s value outwards in time
            check_car_s += ((double)prev_size*.02*check_speed);
            
            // Calculate cost according to next state
            //  "cost functions"
            //  + collision cost
            //  + buffer cost
            //  + inefficiency cost
            for (int j=0; j < states.size(); j++) {
              // set lane coefficient
              int lane_coefficient = 0;
              if (states[j].compare("KL") == 0) {
                lane_coefficient = lane;
              } else if ((states[i].compare("PLCL") == 0) ||
                         (states[i].compare("LCL") == 0)) {
                lane_coefficient = lane - 1;
                // Check current lane
                if (lane == 0) {
                  temp_cost += 100;
                }
              } else if ((states[i].compare("PLCR") == 0) ||
                         (states[i].compare("LCR") == 0)) {
                lane_coefficient = lane + 1;
                // Check current lane
                if (lane == 2) {
                  temp_cost += 100;
                }
              }
              
              if (d < (2+4*lane_coefficient+2) && d > (2+4*lane_coefficient-2)) {
                if (fabs(check_car_s-car_s)  < 30) {
                  // buffer cost
                  too_close = true;
                  temp_cost += 2.0/(1+exp(-2*VEHICLE_RADIUS/fabs(check_car_s-car_s)))-1.0;
                } else if (fabs(check_car_s-car_s)  < 2*VEHICLE_RADIUS) {
                  // colision cost
                  too_close = true;
                  temp_cost += 1.0;
                }
              }
              // inefficiency cost
              temp_cost += (2.0*49.5-check_speed-car_speed)/49.5;
              
              if (temp_cost < best_cost) {
                best_cost = temp_cost;
                best_idx = j;
              }
            }
          }
          
          // Select state by update lane & ref_vel & curr_state
          if (states[best_idx].compare("LCL") == 0) {
            lane -= 1;
          } else if (states[best_idx].compare("LCR") == 0) {
            lane += 1;
          }
          
          curr_state = states[best_idx];
          
          if (too_close) {
            ref_vel -= .224;
          } else if (ref_vel < 49.5) {
            ref_vel += .224;
          }
          
          // END: FSM and cost function

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interoplate these waypoints with a spline and fill it in with more points that control speed
          vector<double> ptsx;
          vector<double> ptsy;
          
          // reference x, y, yaw states
          // either we will reference the starting point as where the car is or at the previous paths end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          // if previous size is almost empty, use the car as starting reference
          if (prev_size < 2) {
            // Use two points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else {
            // Redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            // Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          // In Frenet add evenly 30m spaced points ahead of the starting reference
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for (int i = 0; i < ptsx.size(); i++) {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          
          // create a spline
          tk::spline s;
          
          // set (x, y) points to the spline
          s.set_points(ptsx, ptsy);
          
          // Define the actual (x,y) points we will use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // Start with all of the previous path points from last time
          for (int i=0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline point so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          
          double x_add_on = 0;
          
          // Fill up the reset of our path planner after filling it with previous points, 
          // here we will always output 50 points
          for (int i=1; i <= 50-previous_path_x.size(); i++) {
            // 0.02 for 0.02 seconds, 2.24 is an unit converter from MPH to meter per hour
            double N = (target_dist/(.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)  / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to normal after rotating it eariler
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
      
          /**
           * END
           */

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
