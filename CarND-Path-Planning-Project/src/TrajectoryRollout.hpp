#ifndef __TRAJ_ROLL__
#define __TRAJ_ROLL__
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include <stdint.h>
#include "MathHelperTraj.hpp"
#include "udacityHelpers.hpp"

#define TIME_HORIZON (3.)
#define NUM_POINTS (30)
#define V_MAX (48.)
#define DT (1. / 50.)
#define MAX_SPEED_MPS (22.352) // 50 mph.
#define FOLLOW_DIST (MAX_SPEED_MPS * 2.0)
#define DEFAULT_TARGET_LANE (1)
#define SWITCH_LANE_SPEEDUP_MIN (3.)

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
    double v_t = 0;
    double a_t = 0;
    double j_t = 0;
    TrajPointFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double x);
    TrajPointFrenet(double s, double d);
    bool checkConstraints();
    std::vector<double> toXY(WayPoints wp);
};

class TrajectoryFrenet {
private:
    bool valid = true;
    std::vector<double> x;
    std::vector<double> y;
    double dx = DT;
    void getXY_local(WayPoints wp);
public:
    std::vector<TrajPointFrenet> fr;
    TrajectoryFrenet() {};
    TrajectoryFrenet(double s, double d);
    TrajectoryFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double dt, uint32_t n, WayPoints wp);
    uint32_t size() {return fr.size();};
    void getXY(std::vector<double> & x, std::vector<double> & y);
    bool checkConstraints() {return valid;};
    void print();
};

class Trajectory {
public:
    TrajectoryFrenet fr;
    std::vector<double> x;
    std::vector<double> y;
    Trajectory(VehicleState state, TargetState tgt, TrajectoryFrenet lastTraj, WayPoints wp);
    bool checkConstraints() {return fr.checkConstraints();};
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
    double relSpeed = 0;
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
    std::vector<OtherVehicleState> result;
    std::vector<uint32_t> LoI;
    std::vector<bool> laneIsSafe = std::vector<bool>(3,true);
    std::vector<OtherVehicleState> closestVeh = std::vector<OtherVehicleState>(3);
};

class Roller {
private:
    uint32_t targetLane = DEFAULT_TARGET_LANE;
    void singleLaneFollower(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp);
    void LoIDriver(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp);
    void fastDriver(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp);
public:
    std::vector<Trajectory> trajs;
    Roller(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp);
    uint32_t bestTraj(std::vector<double> & x, std::vector<double> & y, TrajectoryFrenet & tf);

};

#endif