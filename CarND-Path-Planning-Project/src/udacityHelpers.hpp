#ifndef __UDACITY_HELPERS__
#define __UDACITY_HELPERS__

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
#include "spline.h"

using json = nlohmann::json;

#define NUM_MAP_PTS (12000)
#define MAX_S (6945.554)

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double MPH2MPS(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
// else the empty std::string "" will be returned.
std::string hasData(std::string s);

class WayPoints {
public:
  std::vector<double> w_x;
  std::vector<double> w_y;
  std::vector<double> w_s;
  std::vector<double> w_dx;
  std::vector<double> w_dy;
  tk::spline s_x;
  tk::spline s_y;
  tk::spline s_dx;
  tk::spline s_dy;
  WayPoints();
  std::vector<double> getXY(double s, double d);
  std::vector<double> getFrenet(double x, double y, double theta);
};

class VehicleState {
public:
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  // Previous path data given to the Planner
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  std::vector<std::vector<double> > sensor_fusion;

  VehicleState(json j, std::vector<double> px, std::vector<double> py, std::vector<std::vector<double> > sf);
};

#endif