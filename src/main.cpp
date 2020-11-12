#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include <cstdio>
#include <cstdlib>
#include <math.h>
#include <thread>
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using tk::spline;

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
  // Using stringstream to parse the csv file and loadup the corresponding data containers
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
  
  // Defining start lane - 0 far left, incrementing for right lanes
  int lane = 1; // second lane counting from far left 
  
  // Refernce target velocity
  double v_ref = 0; // mph
  int LCR_count = 0; // To penalize right lane changes
  int LCL_count = 0;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &v_ref, &LCR_count, &LCL_count]
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
          //   of the road. It is a vector<vector> with outer being the car and inner vector being its parameters
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          int prevpath_size = previous_path_y.size(); // Last path the car was following before calculating new trajectory points in this step
          
          //  If there exists a non empty previous path points list
          // we initiate car_s to the last s of path points
          if(prevpath_size > 0){
            car_s = end_path_s;
          }
          
          bool too_close = false; // collision avoidance flag
          bool emergency = false;
          bool LCL = false; // Lane change left 
          bool LCR = false; // Lane change right
          bool left_empty = false;
          bool right_empty = false;
          bool mid_empty = false;
          
          
          //find safe v_ref
          for (int i = 0; i < sensor_fusion.size(); ++i)
          {
            // if car is in the same lane as ego
            float d = sensor_fusion[i][6];
            if(d<(2+4*lane+2) && d >(2+4*lane-2))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              
              //If using previous path points, we need to look at future path point
              // For this we project s values out in time
              check_car_s += ((double)prevpath_size*0.02*check_speed); 
              // checking if we are close to other car
              
              if((check_car_s > car_s) && ((check_car_s - car_s)<20)){
                emergency = true;
              }
              
              if((check_car_s > car_s) && ((check_car_s - car_s)<30)){
                too_close = true;
                double min = 20; // safe distance between ego and other vehicles to perform lane change     
                // Initializing closest vehicle in each lane to ego
                double min_left = 10000.0;
                double min_right = 10000.0;
                double min_mid = 10000.0;
                
                for (int j = 0; j < sensor_fusion.size(); ++j){// checking for empty lanes                
                  float d_next = sensor_fusion[j][6];
                  double s_next = sensor_fusion[j][5];
                  double x_next = sensor_fusion[j][3];
                  double y_next = sensor_fusion[j][4];
                  double v_next = sqrt(x_next*x_next + y_next*y_next);
                  s_next += ((double)prevpath_size*0.02*v_next);   
                  double dist = fabs(car_s - s_next);
                  
                  // Updating closest vehicle distance to ego in each lane
                  if((d_next<4 && d_next>0)){
                    if(min_left > dist){
                      min_left = dist;
                    }
                  }
                  
                  if((d_next<8 && d_next>4)){
                    if(min_mid > dist){
                      min_mid = dist;
                    }
                  }   
                  
                  if((d_next<12 && d_next>8)){
                    if(min_right > dist){
                      min_right = dist;
                    }
                  }                    
                } // end of empty lane checking for loop    
                
                // If distance between ego and other cars is high enough, lane considered empty
                if(min_left > min){
                  left_empty = true;
                }
                
                if(min_mid > min){
                  mid_empty = true;
                }
                
                if(min_right > min){
                  right_empty = true;
                }                
                
              }//closeness check loop end
              
            }// same lane as ego
            
          }
          
          if(emergency){
            v_ref -= 2*0.224;
          }
          
            if(too_close){
              v_ref -= 0.5*0.224;
            }
            else if(v_ref < 49.5){
              v_ref += 2*0.224;
            }
          
            if(too_close){
              if(lane==0){
                if(mid_empty && (LCR_count>100)){//count and check mid is empty
                  std::cout<<"Lane Change Right"<<std::endl;
                  LCR = true;
                  LCR_count = 0;
                }
                else{
                  LCR_count += 1;
                }                  
              }

              if(lane==1){
                if(left_empty && (LCL_count>30)){
                  LCL = true;
                  LCL_count = 0;
                }
                else if(right_empty && (LCR_count>100)){// else count and check right is empty
                  LCR = true;
                  LCR_count = 0;
                  std::cout<<"Lane Change Right"<<std::endl;
                }
                else{
                  LCR_count += 1;
                  LCL_count += 1;
                }
              }

              if(lane==2){
                if(left_empty && (LCL_count>30)){
                  LCL = true;
                  LCL_count = 0;
                }
                else{
                  LCL_count += 1;
                }
              }
            }

            if(LCL){
              lane--;
            }

            if(LCR){
              lane++;
            }
          
                  
          // Create a list of sparsely spaced xy waypoints, spaced at 40m
          // These waypoints are interpolated with a spline with more points to maintain speed          
          vector<double> ptsx;
          vector<double> ptsy;
          
          // Refernece states - referring to car state or previous path end points; initializing with car state
          double x_ref = car_x;
          double y_ref = car_y;
          double yaw_ref = deg2rad(car_yaw);
          
          // if prevpath_size is almost empty, use car as starting reference
          if(prevpath_size < 2){
            // Use two points that make path tangent to car; one point is current car state and 
            // other is a point calculated by traveling back in time using the car yaw angle
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(x_ref);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(y_ref);         
          }
          // Else use car previous path end points as refernece points and do the same thing
          else{
            // redefine refernce states 
            x_ref = previous_path_x[prevpath_size-1];
            y_ref = previous_path_y[prevpath_size-1];
            
            double ref_y_prev = previous_path_y[prevpath_size-2];
            double ref_x_prev = previous_path_x[prevpath_size-2];
            yaw_ref = atan2(y_ref-ref_y_prev,x_ref-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(x_ref);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(y_ref);            
          }
          
          // Add evenly spaced frenet points 40m apart ahead of starting refernce 
          vector<double> next_wp0 = getXY(car_s+40, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+80, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+120, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          // At this point the ptsx, ptsy vectors signify 5 points, they are:
          // previous to ref point, ref point, 40m, 80m, 120m from current car state
          
          // Transforming sparsed waypoints list to car's coordinate system
          // This makes it easy for speed based interpolation at a later step
          for (int i=0; i < ptsx.size(); i++){
            // shifting ar ref angle to 0
            double shift_x = ptsx[i]-x_ref;
            double shift_y = ptsy[i]-y_ref;
            
            ptsx[i] = (shift_x*cos(0-yaw_ref)-shift_y*sin(0-yaw_ref));
            ptsy[i] = (shift_x*sin(0-yaw_ref)+shift_y*cos(0-yaw_ref));
          }
          
          //creating a spline
          tk::spline s;
          
          // setting ptsx, ptsy to the spline to generate a smooth trajectory through these points
          s.set_points(ptsx,ptsy);
          
          // Defining actual points that we use for planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          // start by adding all the previously generated path points
          // the size of path points should be chosen appropriately
          // size defines planner robustness to environmental changes and computation necessity
          for (int i=0; i<prevpath_size; ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Breaking up spline points to maintain speed
          // Linearizing the spline to calculate target distance
          double target_x = 40.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
          double x_addon = 0; // origin
          
          // Adding rest of the path planner points
          for (int i=0; i <= 50 - prevpath_size; ++i){
            double N = (target_dist/(0.02*(v_ref/2.24))); // number of points to maintain speed
            double x_pt = x_addon+(target_x)/N;
            double y_pt = s(x_pt);
            
            x_addon = x_pt;
            
            double tempx = x_pt;
            double tempy = y_pt;
            
            // transforming back to map coordinates
            x_pt = (tempx *cos(yaw_ref)-tempy*sin(yaw_ref));
            y_pt = (tempx *sin(yaw_ref)+tempy*cos(yaw_ref));
            
            x_pt += x_ref;
            y_pt += y_ref;
            
            next_x_vals.push_back(x_pt);
            next_y_vals.push_back(y_pt);
          }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
// //           Stay in lane
//           double dist_inc = 0.5;
//           for (int i = 0; i < 50; ++i) {
//             double next_s = car_s+(i+1)*dist_inc;
//             double next_d = 6;
//             vector<double> next_XY = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//             next_x_vals.push_back(next_XY[0]);
//             next_y_vals.push_back(next_XY[1]);
//           }
          
          // Circle
//           double pos_x;
//           double pos_y;
//           double angle;
//           int path_size = previous_path_x.size();

//           for (int i = 0; i < path_size; ++i) {
//             next_x_vals.push_back(previous_path_x[i]);
//             next_y_vals.push_back(previous_path_y[i]);
//           }

//           if (path_size == 0) {
//             pos_x = car_x;
//             pos_y = car_y;
//             angle = deg2rad(car_yaw);
//           } else {
//             pos_x = previous_path_x[path_size-1];
//             pos_y = previous_path_y[path_size-1];

//             double pos_x2 = previous_path_x[path_size-2];
//             double pos_y2 = previous_path_y[path_size-2];
//             angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
//           }

//           double dist_inc = 0.5;
//           for (int i = 0; i < 50-path_size; ++i) {    
//             next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
//             next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
//             pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
//             pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
//           }
          json msgJson;         
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