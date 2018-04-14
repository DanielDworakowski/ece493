#include "TrajectoryRollout.hpp"

#define PRINT(x) std::cout << #x << ": " << x << std::endl;
// 
// Calculate a coordinate in the trajectory and the analytical derivative's there.
TrajPointFrenet::TrajPointFrenet(std::vector<double> traj_s, std::vector<double> traj_d, double x)
{
    s = polyval(traj_s, x, 0);
    s_d = polyval(traj_s, x, 1);
    s_dd = polyval(traj_s, x, 2);
    d = polyval(traj_d, x, 0);
    d_d = polyval(traj_d, x, 1);
    d_dd = polyval(traj_d, x, 2);
    //
    // While these are analytical, they dont match what the simulator thinks they are.
    // This is because the simulator uses finite differences. 
    v_t = norm(d_d, s_d);
    // 
    // Acceleration calculation via finite differences is unstable 
    // probably due to courseness of the map.
    a_t = norm(d_dd, s_dd);
    // j_t = norm(polyval(traj_s, x, 3), polyval(traj_d, x, 3));
}
// 
// Checks the analytical versions of the constaints, does not necessarily match what happens in the simulator.
bool TrajPointFrenet::checkConstraints()
{
    //
    // return !(v_t >= MPH2MPS(V_MAX) || a_t >= 9.9 || j_t >= 9.9);
    return !(v_t >= MPH2MPS(V_MAX) || a_t >= 9.9);
    // return !(a_t >= 7.); // 7 as a safety margin for estimation error. 
}
// 
// Directly set a point (useful for the startup)
TrajPointFrenet::TrajPointFrenet(double s, double d)
{
    this->s = s;
    this->d = d;
}
// 
// Convert to xy frame. 
std::vector<double> TrajPointFrenet::toXY(WayPoints wp)
{
    return wp.getXY(s, d);
}
// 
// A full frenet-frame trajectory. 
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
        valid &= pt.checkConstraints();
        x += dx;
    }
    getXY_local(wp);
}
// 
// Useful if there is a single point only. 
TrajectoryFrenet::TrajectoryFrenet(double s, double d)
{
    fr.push_back(TrajPointFrenet(s,d));
}
// 
// Debugging.
void TrajectoryFrenet::print()
{
    for (uint32_t idx = 0; idx < fr.size(); ++idx) {
        auto pt = fr[idx];
        std::cout << "(s,d,idx): (" << pt.s << ", " << pt.d << ", " << idx << ")\n";
    }
}
// 
// Put the xy coordinates into the local buffer. 
void TrajectoryFrenet::getXY_local(WayPoints wp)
{
    for (uint32_t idx = 0; idx < fr.size(); ++idx) {
        std::vector<double> xy = fr[idx].toXY(wp);
        x.push_back(xy[0]);
        y.push_back(xy[1]);
    }
}
// 
// Return the determined trajectory.
void TrajectoryFrenet::getXY(std::vector<double> & x, std::vector<double> & y)
{
    x = this->x;
    y = this->y;
}
// 
// Generate a trajectory based on desired properties.
Trajectory::Trajectory(VehicleState state, TargetState tgt, TrajectoryFrenet lastTraj, WayPoints wp)
{
    // 
    // Need a point in the Frenet frame to base our new trajectory.
    uint32_t nUnused = state.previous_path_x.size();
    TrajPointFrenet basePoint = lastTraj.fr[lastTraj.size() - nUnused - 1];
    double time = TIME_HORIZON;
    uint32_t n = NUM_POINTS;
    // 
    // Where did the trajectory start -> s.
    double start_s0 = basePoint.s;
    double start_s0_d = basePoint.s_d;
    double start_s0_dd = basePoint.s_dd;
    // 
    // Where did the trajectory start -> d. 
    double start_d0 = basePoint.d;
    double start_d0_d = basePoint.d_d;
    double start_d0_dd = basePoint.d_dd;
    // 
    // End of trajectory in terms of moving forwards. 
    double end_s0 = basePoint.s + tgt.v_f * time; // Integrate final position.
    double end_s0_d = tgt.v_f;
    double end_s0_dd = 0;
    // 
    // How to finish a trajectory in terms of d. 
    double end_d0 = laneNumToM(tgt.lane);
    double end_d0_d = 0;
    double end_d0_dd = 0;
    //
    // Get polynomials based on minimizing jerk.
    std::vector<double> s_coef = minJerkTraj(start_s0, start_s0_d, start_s0_dd, end_s0, end_s0_d, end_s0_dd, time);
    std::vector<double> d_coef = minJerkTraj(start_d0, start_d0_d, start_d0_dd, end_d0, end_d0_d, end_d0_dd, time);
    //
    // Roll out the trajectory and gather xy results.
    fr = TrajectoryFrenet(s_coef, d_coef, DT, n, wp);
    fr.getXY(x, y);
}
// 
// Container for describing the states of the surrounding vehicles.
OtherVehicleState::OtherVehicleState(std::vector<double> state, VehicleState egoState)
{
    // 
    // Based on documnentation.
    id = state[0];
    x = state[1];
    y = state[2];
    v_x = state[3];
    v_y = state[4];
    s = state[5];
    d = state[6];
    // 
    // Maximum speed of the vehicle assuming its following the road. 
    double maxSpeed = std::min(std::sqrt(std::pow(v_x, 2) + std::pow(v_y, 2)), MAX_SPEED_MPS);
    // 
    // How far behind is the vehicle on the road path. 
    reldist =  s - egoState.car_s;
    // 
    // Relative speed difference. 
    relSpeed = maxSpeed - egoState.car_speed;
}
// 
// Based on the current vehicles state, filter out the vehicles that we are not interested in. 
ObservationFilter::ObservationFilter(VehicleState state, std::vector<std::vector<double> > others)
{
    uint32_t curlane = mToLaneNum(state.car_d);
    LoI = getLoI(curlane);
    std::vector<double> closestDist(3,999999); // Should use limits but lazy.
    // 
    // Loop over all vehicles and check if they are close enough to care about. 
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
        //
        // Check if they are too far ahead.
        double maxSpeed = std::min(std::sqrt(std::pow(veh.v_x, 2) + std::pow(veh.v_y, 2)), MAX_SPEED_MPS);
        double relDist = MAX_SPEED_MPS * TIME_HORIZON;
        // 
        // Is the car very far away.
        if (std::abs(veh.reldist) > relDist) {
            continue;
        }
        //
        // If a lane has anything too close then dont consider it.
        double projectedRel = veh.reldist + veh.relSpeed * 2.0;
        //
        // Check if it passed the ego vehicle in simulation.
        bool passesvehicle = (projectedRel > 0 && veh.reldist < 0);
        // 
        // Lane cannot be entered without collision. 
        if (std::abs(veh.reldist) < 5 || std::abs(projectedRel) < 5 || passesvehicle) {
            laneIsSafe[vehLane] = false;
        }
        //
        // Get the distance of the closest car -> speed regulation. 
        if (veh.reldist > -6 && veh.reldist < closestDist[vehLane]) {
            closestDist[vehLane] = veh.reldist;
            closestVeh[vehLane] = veh;
        }
        // 
        // Save results. 
        result.push_back(veh);
    }
}
// 
// Handle doing trajectory rollouts and evaluation.
Roller::Roller(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp)
{
    // singleLaneFollower(state, lastTraj, lastTargetLane, wp);
    fastDriver(state, lastTraj, lastTargetLane, wp);
}
// 
// Different driving behaviour (crosses across multiple lanes at once).
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
    for (uint32_t lnidx = 0; lnidx < filtered.closestVeh.size(); ++lnidx) {
        //
        // Check the speed of all LoI.
        auto curVeh = filtered.closestVeh[lnidx];
        tgtSpeedLane[lnidx] = unrestricted;
        if (filtered.closestVeh[lnidx].id != -1) {
            // 
            // What speed would we go if we entered this lane (P control on follow distance).
            tgtSpeedLane[lnidx] = state.car_speed + 0.2 * (filtered.closestVeh[lnidx].reldist - FOLLOW_DIST);
            if (filtered.closestVeh[lnidx].reldist < 5 || !filtered.laneIsSafe[lnidx]) {
                tgtSpeedLane[lnidx] = -1;
            }
        }
        //
        // Is this lane faster?
        if (tgtSpeedLane[lnidx] > maxSpeed) {
            maxSpeed = tgtSpeedLane[lnidx];
            bestLaneIdx = lnidx;
        }
    }
    //
    // Do we have to skip a lane to get to the target?
    if (std::abs(int32_t(lastTargetLane) - int32_t(bestLaneIdx)) > 1) {
        //
        // If there is a vehicle less than 10m away in the middle stay in the current lane.
        if (!filtered.laneIsSafe[1]) {
            bestLaneIdx = lastTargetLane;
        }
    }
    //
    // Check if it is even worth switching lanes.
    if (maxSpeed - tgtSpeedLane[lastTargetLane] < SWITCH_LANE_SPEEDUP_MIN) {
        bestLaneIdx = lastTargetLane;
    }
    // 
    // Generate a trajectory.
    std::cout << "(maxspeed, bestidx, lastLaneIdx, lastLaneSpeed) = (" << maxSpeed << ", " <<  bestLaneIdx << ", " << lastTargetLane << ", " << tgtSpeedLane[lastTargetLane] << ")\n";
    auto speed = std::min(tgtSpeedLane[bestLaneIdx], v_tgt);
    TargetState tgt(bestLaneIdx, speed);
    targetLane = bestLaneIdx;
    Trajectory traj(state, tgt, lastTraj, wp);
    // 
    // Keep generating trajectories if a constraint is violated. 
    // To make this more realistic, would probably want to have a keep going straight fallback. 
    while (!traj.checkConstraints()) {
        // 
        // Instead of just reducing speed can also try differing trajectories. 
        speed -= 0.25;
        tgt = TargetState(bestLaneIdx, speed);
        traj = Trajectory(state, tgt, lastTraj, wp);
        //
        // If this is a lane change and it doesnt find antrhingf make it go straight.
        if (speed < 2) {
            break;
        }
    }
    trajs.push_back(traj);
}
// 
// Different driver behaviour -> not used. 
void Roller::LoIDriver(VehicleState state, TrajectoryFrenet lastTraj, uint32_t lastTargetLane, WayPoints wp)
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
    auto speed = std::min(tgtSpeedLane[bestLaneIdx], v_tgt);
    TargetState tgt(bestLaneIdx, speed);
    targetLane = bestLaneIdx;
    Trajectory traj(state, tgt, lastTraj, wp);
    while (!traj.checkConstraints()) {
        speed -= 0.25;
        tgt = TargetState(bestLaneIdx, speed);
        traj = Trajectory(state, tgt, lastTraj, wp);
        if (speed < 2) {
            break;
        }
    }
    trajs.push_back(traj);
}
// 
// Only drives in one lane.
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
// 
// Get the selected trajectory. 
uint32_t Roller::getTrajectory(std::vector<double> & x, std::vector<double> & y, TrajectoryFrenet & tf)
{
    x = trajs[0].x;
    y = trajs[0].y;
    tf = trajs[0].fr;
    return targetLane;
}