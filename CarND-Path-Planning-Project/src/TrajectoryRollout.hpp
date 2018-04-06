#ifndef __TRAJ_ROLL__
#define __TRAJ_ROLL__
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include <stdint.h>
#include "udacityHelpers.hpp"

#define TIME_HORIZON (2.)
#define NUM_POINTS (30)
#define DT (1. / 50.)

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
// http://mplab.ucsd.edu/tutorials/minimumJerk.pdf
std::vector<double> minJerkTraj(double x0, double dx0, double ddx0, double xT, double dxT, double ddxT, double T);

inline double laneNumToM(uint32_t laneNum);

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

class Roller {
public:
    std::vector<Trajectory> trajs;
    Roller(VehicleState state, TrajectoryFrenet lastTraj, WayPoints wp);
    void bestTraj(std::vector<double> & x, std::vector<double> & y, TrajectoryFrenet & tf);

};

#endif