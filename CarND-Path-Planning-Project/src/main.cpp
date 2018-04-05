#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "udacityHelpers.hpp"

// for convenience
using json = nlohmann::json;

std::vector<double> minJerkTraj(x0, dx0, ddx0, xT, dxT, ddxT, T) 
{
  T2 
}

// function a = minimumJerk(x0, dx0, ddx0,xT,dxT,ddxT,T)
// % Compute a point to point minimum jerk trajectory
// % x0 dx0 ddx0 are the location, velocity and acceleration at the
// % start point
// % xT dxT ddxT are the target location velocity and acceleration
// % T is the time required to move from the start point
// % to the target point
// %
// % The solution is a 6-D vector of coefficients a
// % The minimum jerk trajectory takes the form
// % x_t = \sum_{k=1}^6 a_k t^(k-1), for 0\leq t \leq T
// %
// % Copyright Javier R. Movellan UCSD 2011
// T2 = T*T; T3 = T2*T;
// T4 = T3*T; T5= T4*T;
// a = zeros(6,1);
// a(1) = x0;
// a(2) = dx0;
// a(3) = ddx0/2;
// b= [T3 T4 T5 ; 3*T2 4*T3 5*T4; 6*T 12* T2 20* T3];
// c = [ xT - a(1) - a(2)*T - a(3)*T2; dxT - a(2) - 2*a(3)*T;
// ddxT - 2*a(3)];
// a(4:6,1)=pinv(b)*c;

int main() 
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal std::vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = std::string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        std::string event = j[0].get<std::string>();
        
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

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            std::vector<double> next_x_vals;
            std::vector<double> next_y_vals;

            int startx = car_x;
            int starty = car_y;
            for (int x = 0; x < 5; ++x) {
              int wpt = NextWaypoint(startx, starty, car_yaw, map_waypoints_x, map_waypoints_y);
              ++wpt;
              if (wpt >= map_waypoints_x.size()) {
                wpt = 0;
              }
              startx = map_waypoints_x[wpt];
              starty = map_waypoints_y[wpt];
              auto fren = getFrenet(startx, starty, car_yaw, map_waypoints_x, map_waypoints_y);
              auto fren_s = fren[0];
              auto fren_d = fren[1] + 2;
              auto xy = getXY(fren_s, fren_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
            }

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            std::cout << "msg: " << msgJson << std::endl;
            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } 
      else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } 
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
  } 
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
