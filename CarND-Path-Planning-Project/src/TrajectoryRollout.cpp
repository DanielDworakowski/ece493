#include "TrajectoryRollout.hpp"

#define PRINT(x) std::cout << #x << ": " << x << std::endl;

TrajPointFrenet::TrajPointFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double x)
{
    s = polyval(traj_s, x);
    s_d = dpolyval(traj_s, x);
    s_dd = ddpolyval(traj_s, x);
    d = polyval(traj_d, x);
    d_d = dpolyval(traj_d, x);
    d_dd = ddpolyval(traj_d, x);
    v_t = norm(d_d, s_d);
    a_t = norm(d_dd, s_dd);
    j_t = norm(dddpolyval(traj_s, x), dddpolyval(traj_d, x));
}

bool TrajPointFrenet::checkConstraints()
{
    //
    // SHould be inverted?
    return !(v_t >= MPH2MPS(V_MAX) || a_t >= 9.9 || j_t >= 9.9);
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

TrajectoryFrenet::TrajectoryFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double dx, uint32_t n, WayPoints wp)
{
    //
    // Starting at 0 should include the previous point.
    double x = 0;
    this->dx = dx;
    //
    // Evaluate the trajectory at each point.
    for (uint32_t idx = 0; idx < n; ++idx) {
        auto pt = TrajPointFrenet(traj_s, traj_d, x);
        fr.push_back(pt);
        // valid &= pt.checkConstraints();
        x += dx;
    }
    getXY_local(wp);
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

void TrajectoryFrenet::getXY_local(WayPoints wp)
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
    //
    // Check constraints in the x-y frame.
    double v_x = 0;
    double v_y = 0;
    double v_t = 0;
    double a_x = 0;
    double a_y = 0;
    double a_t = 0;
    double j_x = 0;
    double j_y = 0;
    double j_t = 0;
    //
    // Loop with forward differences.
    for (uint32_t i = 1; i < fr.size() - 3; ++i) {
        v_x = (-11 * x[i] + 18 * x[i + 1] - 9 * x[i + 2] + 2 * x[i + 3]) / (6*dx);
        v_y = (-11 * y[i] + 18 * y[i + 1] - 9 * y[i + 2] + 2 * y[i + 3]) / (6*dx);
        v_t = norm(v_x, v_y);
        // a_x = (11*x[i-1]-20*x[i]+6*x[i+1]+4*x[i+2]-x[i+3]) / std::pow(dx, 2);
        // a_y = (11*y[i-1]-20*y[i]+6*y[i+1]+4*y[i+2]-y[i+3]) / std::pow(dx, 2);
        // a_t = norm(a_x, a_y);
        // j_x = (-x[i] + x[i + 3] + 3 * x[i + 1] - 3 * x[i + 2]) / std::pow(dx, 3);
        // j_y = (-y[i] + y[i + 3] + 3 * y[i + 1] - 3 * y[i + 2]) / std::pow(dx, 3);
        // j_t = norm(j_x, j_y);
        // valid &= (!(v_t >= MPH2MPS(V_MAX) || a_t >= 9.9 || j_t >= 9.9));
        valid &= (!(v_t >= MPH2MPS(V_MAX)));
    }
}

void TrajectoryFrenet::getXY(std::vector<double> & x, std::vector<double> & y)
{
    x = this->x;
    y = this->y;
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
    fr = TrajectoryFrenet(s_coef, d_coef, DT, n, wp);
    fr.getXY(x, y);
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
    // reldist =  s + (maxSpeed - egoState.car_speed) * TIME_HORIZON - egoState.car_s;
    reldist =  s - egoState.car_s;
}

// [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.
ObservationFilter::ObservationFilter(VehicleState state, std::vector<std::vector<double> > others)
{
    uint32_t curlane = mToLaneNum(state.car_d);
    LoI = getLoI(curlane);
    std::vector<double> closestDist(3,999999); // Should use limits but lazy.

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
        // if (std::abs(veh.s + (maxSpeed - state.car_speed) * TIME_HORIZON - state.car_s) > relDist) {
        //     continue;
        // }
        if (std::abs(veh.s - state.car_s) > relDist) {
            continue;
        }
        // //
        // // Remove anything too far behind.
        // if (veh.s + maxSpeed * TIME_HORIZON < state.car_s) {
        //     continue;
        // }
        //
        // Check if the car ahead is the closest in the LoI.
        if (veh.reldist > -10 && veh.reldist < closestDist[vehLane]) {
            closestDist[vehLane] = veh.reldist;
            closestVeh[vehLane] = veh;
        }
        result.push_back(veh);
    }
}

Roller::Roller(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp)
{
    // singleLaneFollower(state, lastTraj, lastTargetLane, wp);
    fastDriver(state, lastTraj, lastTargetLane, wp);

    // for (uint32_t lnIdx = 0; lnIdx < LoI.size(); ++lnIdx) {
    //     TargetState tgt(LoI[lnIdx], vf_controller);
    //     trajs.push_back(Trajectory(state, tgt, lastTraj, wp));
    // }
}

void Roller::fastDriver(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp)
{
    double tgtspeed = MPH2MPS(V_MAX);
    double unrestricted = MPH2MPS(100);
    //
    // Speed with no obstruction.
    double vf_controller = std::min(tgtspeed, state.car_speed + 0.2 * (tgtspeed - state.car_speed));
    double v_tgt = vf_controller;
    double maxSpeed = -1000;
    uint32_t bestLaneIdx = 100;
    uint32_t curlane = mToLaneNum(state.car_d);
    ObservationFilter filtered(state, state.sensor_fusion);
    std::vector<double> tgtSpeedLane(filtered.closestVeh.size(), -100);
    //
    // Iterate over all LoI.
    for (uint32_t LoIIndex = 0; LoIIndex < filtered.LoI.size(); ++LoIIndex) {
        //
        // Check the speed of all LoI.
        auto loiidx = filtered.LoI[LoIIndex];
        auto curVeh = filtered.closestVeh[loiidx];
        tgtSpeedLane[loiidx] = unrestricted;
        if (filtered.closestVeh[loiidx].id != -1) {
            // PRINT(loiidx)
            // filtered.closestVeh[loiidx].print();
            // PRINT(filtered.closestVeh[loiidx].reldist)
            tgtSpeedLane[loiidx] = state.car_speed + 0.2 * (filtered.closestVeh[loiidx].reldist - FOLLOW_DIST);
            if (filtered.closestVeh[loiidx].reldist < 5) {
                tgtSpeedLane[loiidx] = -1;
            }
        }
        //
        // Is this lane faster?
        if (tgtSpeedLane[loiidx] > maxSpeed) {
            maxSpeed = tgtSpeedLane[loiidx];
            bestLaneIdx = loiidx;
        }
    }
    // std::cout << "(maxspeed, bestidx, lastLaneIdx, lastLaneSpeed) = (" << maxSpeed << ", " <<  bestLaneIdx << ", " << lastTargetLane << ", " << tgtSpeedLane[lastTargetLane] << ")\n";
    //
    // Check if it is even worth switching lanes.
    if (maxSpeed - tgtSpeedLane[lastTargetLane] < SWITCH_LANE_SPEEDUP_MIN) {
        bestLaneIdx = lastTargetLane;
    }
    else {
        std::cout << "Worth switching lanes: " << maxSpeed - tgtSpeedLane[lastTargetLane] << std::endl;
    }
    auto speed = std::min(tgtSpeedLane[bestLaneIdx], v_tgt);
    TargetState tgt(bestLaneIdx, speed);
    targetLane = bestLaneIdx;
    Trajectory traj(state, tgt, lastTraj, wp);
    while (!traj.checkConstraints()) {
        speed -= 1;
        tgt = TargetState(bestLaneIdx, speed);
        traj = Trajectory(state, tgt, lastTraj, wp);
        if (speed < 2) {
            break;
        }
    }
    trajs.push_back(traj);
}

void Roller::singleLaneFollower(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp)
{
    double tgtspeed = MPH2MPS(45);
    double vf_controller = std::min(tgtspeed, state.car_speed + 0.2 * (tgtspeed - state.car_speed));
    double v_tgt = vf_controller;
    uint32_t laneIdx = 1;
    uint32_t curlane = mToLaneNum(state.car_d);
    //
    // Stay in one lane and follow.
    ObservationFilter filtered(state, state.sensor_fusion);
    if (filtered.closestVeh[curlane].id != -1) {
        filtered.closestVeh[curlane].print();
        v_tgt = state.car_speed + 0.2 * (filtered.closestVeh[curlane].reldist - FOLLOW_DIST);
    }
    v_tgt = std::max(0., v_tgt);
    TargetState tgt(laneIdx, v_tgt);
    trajs.push_back(Trajectory(state, tgt, lastTraj, wp));
}

uint32_t Roller::bestTraj(std::vector<double> & x, std::vector<double> & y, TrajectoryFrenet & tf)
{
    x = trajs[0].x;
    y = trajs[0].y;
    tf = trajs[0].fr;
    return targetLane;
}