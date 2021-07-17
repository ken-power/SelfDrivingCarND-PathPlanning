//
// Created by Ken Power on 17/07/2021.
//

#ifndef PATH_PLANNING_WAYPOINTS_H
#define PATH_PLANNING_WAYPOINTS_H

#include "network_utils.h"
#include "distance_utils.h"


// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> & maps_x,
                    const vector<double> & maps_y)
{
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); ++i)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = Distance(x, y, map_x, map_y);

        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> & maps_x,
                 const vector<double> & maps_y)
{
    int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * PI() - angle, angle);

    if(angle > PI() / 2)
    {
        ++closestWaypoint;
        if(closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

#endif //PATH_PLANNING_WAYPOINTS_H
