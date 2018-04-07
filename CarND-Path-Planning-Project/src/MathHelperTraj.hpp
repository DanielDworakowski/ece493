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
inline std::vector<double> minJerkTraj(double x0, double dx0, double ddx0, double xT, double dxT, double ddxT, double T)
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

inline double norm(double v1, double v2)
{
    return std::sqrt(std::pow(v1, 2) + std::pow(v2, 2));
}

inline double laneNumToM(uint32_t laneNum)
{
    double ret = 0;
    if (laneNum > 2) {
        throw "Outside of the max lanes!";
    }
    //
    // Noticed that if car goes too far to the right it randomly fails.
    ret = 2 + laneNum * 4;
    if (laneNum == 2) {
        ret -= 0.2;
    }
    return ret;
}

inline uint32_t mToLaneNum(double m)
{
    // if (m < 0 || m > 10) {
    //     std::cerr << "Measurement outside of highway!\n";
    // }
    uint32_t ret = std::round(std::abs(m - 2) / 4);
    return ret;
}

inline double polyval(std::vector<double> a, double x)
{
    double ret = 0;
    for (uint32_t idx = 0; idx < a.size(); ++idx) {
        ret += a[idx] * pow(x, idx);
    }
    return ret;
}

inline double dpolyval(std::vector<double> a, double x)
{
    double ret = 0;
    for (uint32_t idx = 1; idx < a.size(); ++idx) {
        ret += idx * a[idx] * pow(x, idx - 1);
    }
    return ret;
}

inline double ddpolyval(std::vector<double> a, double x)
{
    double ret = 0;
    for (uint32_t idx = 2; idx < a.size(); ++idx) {
        ret += idx * (idx - 1) * a[idx] * pow(x, idx - 2);
    }
    return ret;
}

inline double dddpolyval(std::vector<double> a, double x)
{
    double ret = 0;
    for (uint32_t idx = 3; idx < a.size(); ++idx) {
        ret += idx * (idx - 1) * (idx - 2) * a[idx] * pow(x, idx - 3);
    }
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

// inline std::vector<uint32_t> getLoI(uint32_t curlane)
// {

//     std::vector<uint32_t> ret;
//     ret.push_back(0);
//     ret.push_back(1);
//     ret.push_back(2);
//     // ret.push_back(curlane);
//     // //
//     // // Return the lanes that are immediately next to the current.
//     // switch(curlane) {
//     //     case 0:
//     //         ret.push_back(1);
//     //         break;
//     //     case 1:
//     //         ret.push_back(0);
//     //         ret.push_back(2);
//     //         break;
//     //     case 2:
//     //         ret.push_back(1);
//     //         break;
//     //     default:
//     //         throw "Invalid current lane";
//     // }
//     return ret;
// }

#endif