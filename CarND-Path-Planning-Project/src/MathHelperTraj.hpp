#ifndef __MATH_HELPER_TRAJ__
#define __MATH_HELPER_TRAJ__
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/QR"
#include <stdint.h>
#include <vector>

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

double laneNumToM(uint32_t laneNum);
uint32_t mToLaneNum(double m);
std::vector<uint32_t> getLoI(uint32_t curlane);
double polyval(std::vector<double> a, double x);
double dpolyval(std::vector<double> a, double x);
double ddpolyval(std::vector<double> a, double x);

#endif