//
// Created by Ken Power on 17/07/2021.
//

#ifndef PATH_PLANNING_WAYPOINTS_H
#define PATH_PLANNING_WAYPOINTS_H

#include "network_utils.h"
#include "distance_utils.h"


struct WaypointData
{
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
};

/**
 *
 * @param waypoint_data the waypoint data for the highway
 * @param data_row a row of data for a car on the highway; the format for each car is: `[ id, x, y, vx, vy, s, d]`.
*
 */
void PopulateWaypointsData(WaypointData & waypoint_data, const string & data_row)
{
    istringstream iss(data_row);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;

    waypoint_data.map_waypoints_x.push_back(x);
    waypoint_data.map_waypoints_y.push_back(y);
    waypoint_data.map_waypoints_s.push_back(s);
    waypoint_data.map_waypoints_dx.push_back(d_x);
    waypoint_data.map_waypoints_dy.push_back(d_y);
}


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
