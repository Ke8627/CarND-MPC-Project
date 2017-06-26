#ifndef UTIL_H
#define UTIL_H

#include "Eigen-3.3/Eigen/Core"

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

double polyeval(const Eigen::VectorXd& coeffs, double x);
Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order);

#endif /* UTIL_H */
