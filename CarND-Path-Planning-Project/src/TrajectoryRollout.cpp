#include "TrajectoryRollout.hpp"

#define PRINT(x) std::cout << #x << ": " << x << std::endl;

std::vector<double> minJerkTraj(double x0, double dx0, double ddx0, double xT, double dxT, double ddxT, double T)
{
    double T2 = std::pow(T,2);
    double T3 = std::pow(T,3);
    double T4 = std::pow(T,4);
    double T5 = std::pow(T,5);
    Eigen::MatrixXd coef(6,1);
    coef(0) = x0;
    coef(1) = dx0;
    coef(2) = ddx0 / 2;
    Eigen::MatrixXd b(3,3);
    b <<    T3, T4, T5,
            3*T2, 4*T3, 5*T4,
            6*T, 12*T2, 20*T3;
    Eigen::MatrixXd c(3,1);
    c <<    (xT - coef(0) - coef(1) * T - coef(2)*T2),
            (dxT - coef(1) - 2*coef(2)*T),
            (ddxT - 2*coef(2));
    Eigen::MatrixXd inv = b.inverse();
    coef.block(3,0,3,1) = inv * c;
    std::vector<double> ret(coef.data(), coef.data() + coef.rows() * coef.cols());
    return ret;
}


inline std::vector<uint32_t> getLoI(uint32_t curlane)
{
    std::vector<uint32_t> ret;
    ret.push_back(curlane);
    //
    // Return the lanes that are immediately next to the current.
    switch(curlane) {
        case 0:
            ret.push_back(1);
            break;
        case 1:
            ret.push_back(0);
            ret.push_back(2);
            break;
        case 2:
            ret.push_back(1);
            break;
        default:
            throw "Invalid current lane";
    }
    return ret;
}

inline double laneNumToM(uint32_t laneNum)
{
    if (laneNum > 2) {
        throw "Outside of the max lanes!";
    }
    return 2 + laneNum * 4;
}

inline uint32_t mToLaneNum(double m)
{
    // if (m < 0 || m > 10) {
    //     std::cerr << "Measurement outside of highway!\n";
    // }
    uint32_t ret = std::round(std::abs(m - 2) / 4);
    return ret;
}

double polyval(std::vector<double> a, double x)
{
    double ret = 0;
    for (uint32_t idx = 0; idx < a.size(); ++idx) {
        ret += a[idx] * pow(x, idx);
    }
    return ret;
}

double dpolyval(std::vector<double> a, double x)
{
    double ret = 0;
    for (uint32_t idx = 1; idx < a.size(); ++idx) {
        ret += idx * a[idx] * pow(x, idx - 1);
    }
    return ret;
}

double ddpolyval(std::vector<double> a, double x)
{
    double ret = 0;
    for (uint32_t idx = 2; idx < a.size(); ++idx) {
        ret += idx * (idx - 1) * a[idx] * pow(x, idx - 2);
    }
    return ret;
}

TrajPointFrenet::TrajPointFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double x)
{
    s = polyval(traj_s, x);
    s_d = dpolyval(traj_s, x);
    s_dd = ddpolyval(traj_s, x);
    d = polyval(traj_d, x);
    d_d = dpolyval(traj_d, x);
    d_dd = ddpolyval(traj_d, x);
}

TrajPointFrenet::TrajPointFrenet(double s, double d)
{
    this->s = s;
    this->d = d;
}

std::vector<double> TrajPointFrenet::toXY(WayPoints wp)
{
    return wp.getXY(s, d);
}

TrajectoryFrenet::TrajectoryFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double dx, uint32_t n)
{
    //
    // Starting at 0 should include the previous point.
    double x = 0;
    //
    // Evaluate the trajectory at each point.
    for (uint32_t idx = 0; idx < n; ++idx) {
        fr.push_back(TrajPointFrenet(traj_s, traj_d, x));
        x += dx;
    }
}

TrajectoryFrenet::TrajectoryFrenet(double s, double d)
{
    fr.push_back(TrajPointFrenet(s,d));
}

void TrajectoryFrenet::print()
{
    for (uint32_t idx = 0; idx < fr.size(); ++idx) {
        auto pt = fr[idx];
        std::cout << "(s,d,idx): (" << pt.s << ", " << pt.d << ", " << idx << ")\n";
    }
}

void TrajectoryFrenet::getXY(std::vector<double> & x, std::vector<double> & y, WayPoints wp, VehicleState state)
{
    for (uint32_t idx = 0; idx < fr.size(); ++idx) {
        std::vector<double> xy = fr[idx].toXY(wp);
        x.push_back(xy[0]);
        y.push_back(xy[1]);
        //
        // Debugging.
        // auto sd = wp.getFrenet(xy[0], xy[1], state.car_yaw);
        // std::cout << "(s,d,idx): (" << sd[0] << ", " << sd[1] << ", " << idx << ")\n";
    }
}

Trajectory::Trajectory(VehicleState state, TargetState tgt, TrajectoryFrenet lastTraj, WayPoints wp)
{
    uint32_t nUnused = state.previous_path_x.size();
    TrajPointFrenet basePoint = lastTraj.fr[lastTraj.size() - nUnused - 1];
    // auto minJerkTraj(double x0, double dx0, double ddx0, double xT, double dxT, double ddxT, double T)
    double start_s0 = basePoint.s;
    double start_s0_d = basePoint.s_d;
    double start_s0_dd = basePoint.s_dd;
    double start_d0 = basePoint.d;
    double start_d0_d = basePoint.d_d;
    double start_d0_dd = basePoint.d_dd;
    double end_s0 = basePoint.s + tgt.v_f * TIME_HORIZON; // Integrate final position.
    double end_s0_d = tgt.v_f;
    double end_s0_dd = 0; //Always 0.
    double end_d0 = laneNumToM(tgt.lane);
    double end_d0_d = 0; //Always 0.
    double end_d0_dd = 0; //Always 0.
    //
    // Get polynomials.
    std::vector<double> s_coef = minJerkTraj(start_s0, start_s0_d, start_s0_dd, end_s0, end_s0_d, end_s0_dd, TIME_HORIZON);
    std::vector<double> d_coef = minJerkTraj(start_d0, start_d0_d, start_d0_dd, end_d0, end_d0_d, end_d0_dd, TIME_HORIZON);
    uint32_t n = NUM_POINTS;
    //
    // Roll out the trajectory.
    fr = TrajectoryFrenet(s_coef, d_coef, DT, n);
    fr.getXY(x, y, wp, state);
}

OtherVehicleState::OtherVehicleState(std::vector<double> state, VehicleState egoState)
{
    id = state[0];
    x = state[1];
    y = state[2];
    v_x = state[3];
    v_y = state[4];
    s = state[5];
    d = state[6];
    double maxSpeed = std::min(std::sqrt(std::pow(v_x, 2) + std::pow(v_y, 2)), MAX_SPEED_MPS);
    reldist =  s + (maxSpeed - egoState.car_speed) * TIME_HORIZON - egoState.car_s;
}

// [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
ObservationFilter::ObservationFilter(VehicleState state, std::vector<std::vector<double> > others)
{
    uint32_t curlane = mToLaneNum(state.car_d);
    auto LoI = getLoI(curlane);
    double closestDist = 999999; // Should use limits but lazy.

    for (uint32_t vehidx = 0; vehidx < others.size(); ++vehidx) {
        OtherVehicleState veh(others[vehidx], state);
        //
        // Assumption checking.
        if (veh.s > MAX_S) {
            throw "PAST MAX_S";
        }
        //
        // Check if car is in LoI.
        uint32_t vehLane = mToLaneNum(veh.d);
        if (std::find(LoI.begin(), LoI.end(), vehLane) == LoI.end()) {
            continue;
        }
        //
        // Check if they are too far ahead.
        double maxSpeed = std::min(std::sqrt(std::pow(veh.v_x, 2) + std::pow(veh.v_y, 2)), MAX_SPEED_MPS);
        double relDist = MAX_SPEED_MPS * 2;
        if (std::abs(veh.s + (maxSpeed - state.car_speed) * TIME_HORIZON - state.car_s) > relDist) {
            continue;
        }
        // //
        // // Remove anything too far behind.
        // if (veh.s + maxSpeed * TIME_HORIZON < state.car_s) {
        //     continue;
        // }
        //
        // Check if the car ahead is the closest in the LoI.
        if (veh.reldist > 0 && veh.reldist < closestDist) {
            closestDist = veh.reldist;
            closestVeh = veh;
        }
        result.push_back(veh);
    }
}

Roller::Roller(VehicleState state, TrajectoryFrenet lastTraj, WayPoints wp)
{
    // double tgtspeed = MPH2MPS(45);
    double tgtspeed = MPH2MPS(45);
    double vf_controller = std::min(tgtspeed, state.car_speed + 0.2 * (tgtspeed - state.car_speed));
    double v_tgt = vf_controller;
    uint32_t laneIdx = 1;
    uint32_t curlane = mToLaneNum(state.car_d);
    auto LoI = getLoI(curlane);

    ObservationFilter filtered(state, state.sensor_fusion);
    if (filtered.closestVeh.id != -1) {
        filtered.closestVeh.print();
        PRINT(filtered.closestVeh.reldist)
        v_tgt = state.car_speed + 0.2 * (filtered.closestVeh.reldist - FOLLOW_DIST);
    }
    PRINT(v_tgt)
    v_tgt = std::max(0., v_tgt);
    TargetState tgt(laneIdx, v_tgt);
    trajs.push_back(Trajectory(state, tgt, lastTraj, wp));

    // for (uint32_t lnIdx = 0; lnIdx < LoI.size(); ++lnIdx) {
    //     TargetState tgt(LoI[lnIdx], vf_controller);
    //     trajs.push_back(Trajectory(state, tgt, lastTraj, wp));
    // }
}


void Roller::bestTraj(std::vector<double> & x, std::vector<double> & y, TrajectoryFrenet & tf)
{
    x = trajs[0].x;
    y = trajs[0].y;
    tf = trajs[0].fr;
}