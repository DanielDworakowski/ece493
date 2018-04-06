#include "MathHelperTraj.hpp"

double laneNumToM(uint32_t laneNum)
{
    if (laneNum > 2) { 
        throw "Outside of the max lanes!";
    }
    return 2 + laneNum * 4;
}

uint32_t mToLaneNum(double m)
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

std::vector<uint32_t> getLoI(uint32_t curlane)
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