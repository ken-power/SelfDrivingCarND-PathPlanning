//
// Created by Ken Power on 17/07/2021.
//

#ifndef PATH_PLANNING_DISTANCE_UTILS_H
#define PATH_PLANNING_DISTANCE_UTILS_H

#include <cmath>
#include <string>
#include <vector>


// For converting back and forth between radians and degrees.
constexpr double PI()
{
    return M_PI;
}

double Degrees2Radians(double x)
{
    return x * PI() / 180;
}

double Radians2Degrees(double x)
{
    return x * 180 / PI();
}

// Calculate distance between two points
double Distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

#endif //PATH_PLANNING_DISTANCE_UTILS_H
