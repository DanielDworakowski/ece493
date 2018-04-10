#include "udacityHelpers.hpp"

#define DEBUG_LINE std::cout << __LINE__ << std::endl;

// 
// Wrap the waypoints of the map.
WayPoints::WayPoints()
{
  std::string map_file_ = "../data/highway_map.csv";
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  std::vector<double> temp_x;
  std::vector<double> temp_y;
  std::vector<double> temp_s;
  std::vector<double> temp_dx;
  std::vector<double> temp_dy;

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
    temp_x.push_back(x);
    temp_y.push_back(y);
    temp_s.push_back(s);
    temp_dx.push_back(d_x);
    temp_dy.push_back(d_y);
  }
  double max_s = MAX_S;
  // 
  // Wrap around back to the beginning for splining.
  temp_x.push_back(temp_x[0]);
  temp_y.push_back(temp_y[0]);
  temp_s.push_back(max_s);
  temp_dx.push_back(temp_dx[0]);
  temp_dy.push_back(temp_dy[0]);
  // 
  // Create generate the spline for the entire map.
  s_x.set_points(temp_s, temp_x);
  s_y.set_points(temp_s, temp_y);
  s_dx.set_points(temp_s, temp_dx);
  s_dy.set_points(temp_s, temp_dy);
  double cur_s = 0;
  uint32_t npts = NUM_MAP_PTS;
  double ds = max_s / npts;
  // 
  // Iterate over the map and generate more data points. 
  for (uint32_t idx = 0; idx < npts; ++idx) {
    w_s.push_back(cur_s);
    w_x.push_back(s_x(cur_s));
    w_y.push_back(s_y(cur_s));
    w_dx.push_back(s_dx(cur_s));
    w_dy.push_back(s_dy(cur_s));
    cur_s += ds;
  }
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> WayPoints::getXY(double s, double d)
{
  int prev_wp = -1;
  // 
  // Wrap s back around.
  double wraps = std::floor(s / MAX_S);
  s -= wraps * MAX_S;
  while(s > w_s[prev_wp+1] && (prev_wp < (int)(w_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%w_x.size();

  double heading = atan2((w_y[wp2]-w_y[prev_wp]),(w_x[wp2]-w_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-w_s[prev_wp]);

  double seg_x = w_x[prev_wp]+seg_s*cos(heading);
  double seg_y = w_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

std::vector<double> WayPoints::getFrenet(double x, double y, double theta)
{
  int next_wp = NextWaypoint(x,y, theta, w_x,w_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = w_x.size()-1;
  }

  double n_x = w_x[next_wp]-w_x[prev_wp];
  double n_y = w_y[next_wp]-w_y[prev_wp];
  double x_x = x - w_x[prev_wp];
  double x_y = y - w_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-w_x[prev_wp];
  double center_y = 2000-w_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }
  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(w_x[i],w_y[i],w_x[i+1],w_y[i+1]);
  }
  frenet_s += distance(0,0,proj_x,proj_y);
  return {frenet_s,frenet_d};

}
// 
// Wrap the ego vehicle's state. 
VehicleState::VehicleState(json j, std::vector<double> px, std::vector<double> py, std::vector<std::vector<double> > sf)
{
  car_x = j[1]["x"];
  car_y = j[1]["y"];
  car_s = j[1]["s"];
  car_d = j[1]["d"];
  car_yaw = j[1]["yaw"];
  car_speed = MPH2MPS(j[1]["speed"]);
  // Previous path data given to the Planner
  previous_path_x = px;
  previous_path_y = py;
  // Previous path's end s and d values
  end_path_s = j[1]["end_path_s"];
  end_path_d = j[1]["end_path_d"];
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  sensor_fusion = sf;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double MPH2MPS(double x) {return x * 0.44704; }

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()){
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }
  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }
  frenet_s += distance(0,0,proj_x,proj_y);
  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
// else the empty std::string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

