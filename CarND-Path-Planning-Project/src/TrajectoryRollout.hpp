#ifndef __TRAJ_ROLL__
#define __TRAJ_ROLL__
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include <stdint.h>
#include "MathHelperTraj.hpp"
#include "udacityHelpers.hpp"

#define TIME_HORIZON (2.)
#define NUM_POINTS (30)
#define DT (1. / 50.)
#define MAX_SPEED_MPS (22.352) // 50 mph.
#define FOLLOW_DIST (MAX_SPEED_MPS * TIME_HORIZON)

class TargetState {
public:
    uint32_t lane = 0;
    double v_f = 0;
    TargetState(uint32_t lane, double v_f)
        : lane(lane)
        , v_f(v_f) {};
};

class TrajPointFrenet {
public:
    double s = 0.;
    double s_d = 0.;
    double s_dd = 0.;
    double d = 0.;
    double d_d = 0.;
    double d_dd = 0.;
    TrajPointFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double x);
    TrajPointFrenet(double s, double d);
    std::vector<double> toXY(WayPoints wp);
};

class TrajectoryFrenet {
public:
    std::vector<TrajPointFrenet> fr;
    TrajectoryFrenet() {};
    TrajectoryFrenet(double s, double d);
    TrajectoryFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double dt, uint32_t n);
    void getXY(std::vector<double> & x, std::vector<double> & y, WayPoints wp, VehicleState state);
    uint32_t size() {return fr.size();};
    void print();
};

class Trajectory {
public:
    TrajectoryFrenet fr;
    std::vector<double> x;
    std::vector<double> y;
    Trajectory(VehicleState state, TargetState tgt, TrajectoryFrenet lastTraj, WayPoints wp);
};

struct OtherVehicleState {
    int32_t id = -1;
    double x = 0;
    double y = 0;
    double v_x = 0;
    double v_y = 0;
    double s = 0;
    double d = 0;
    double reldist = 0;
    OtherVehicleState(std::vector<double>, VehicleState);
    OtherVehicleState() {};
    void print()
    {
        std::cout << "(id, s, d) = (" << id << ", " <<  s << ", " << d << ")\n";
    };
};

class ObservationFilter {
public:
    ObservationFilter(VehicleState state, std::vector<std::vector<double> > others);
    std::vector<OtherVehicleState > result;
    OtherVehicleState closestVeh;
};

class Roller {
private:

public:
    std::vector<Trajectory> trajs;
    Roller(VehicleState state, TrajectoryFrenet lastTraj, WayPoints wp);
    void bestTraj(std::vector<double> & x, std::vector<double> & y, TrajectoryFrenet & tf);

};

#endif