#ifndef HELPER_FUNCTION_H
#define HELPER_FUNCTION_H

#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

int GetLaneNo(double d);
double distance(double x1, double y1, double x2, double y2);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getSdotDdot(double sinit, double dinit, double xinit, double yinit, double vx, double vy, float duration, const vector<double> &maps_x, const vector<double> &maps_y);


#endif
