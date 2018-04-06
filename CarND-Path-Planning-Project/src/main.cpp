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
#include "TrajectoryRollout.hpp"

// for convenience
using json = nlohmann::json;

int main()
{
  uWS::Hub h;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  WayPoints wp;
  TrajectoryFrenet prev_tf;
  bool init = false;

  h.onMessage([&wp, &prev_tf, &init](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = std::string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        json j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          VehicleState state(j, j[1]["previous_path_x"], j[1]["previous_path_y"], j[1]["sensor_fusion"]);
          if (!init) {
            prev_tf = TrajectoryFrenet(state.car_s, state.car_d);
            init = true;
          }

          json msgJson;

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;
          Roller roll(state, prev_tf, wp);
          roll.bestTraj(next_x_vals, next_y_vals, prev_tf);


          // int startx = state.car_x;
          // int starty = state.car_y;
          // for (int x = 0; x < 5; ++x) {
          //   int wpt = NextWaypoint(startx, starty, state.car_yaw, wp.w_x, wp.w_y);
          //   ++wpt;
          //   if (wpt >= wp.w_x.size()) {
          //     wpt = 0;
          //   }
          //   startx = wp.w_x[wpt];
          //   starty = wp.w_y[wpt];
          //   auto fren = getFrenet(startx, starty, state.car_yaw, wp.w_x, wp.w_y);
          //   auto fren_s = fren[0];
          //   auto fren_d = fren[1] + 2;
          //   auto xy = getXY(fren_s, fren_d, wp.w_s, wp.w_x, wp.w_y);
          //   next_x_vals.push_back(xy[0]);
          //   next_y_vals.push_back(xy[1]);
          // }

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
