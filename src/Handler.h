//
// Created by Ken Power on 17/07/2021.
//

#ifndef PATH_PLANNING_HANDLER_H
#define PATH_PLANNING_HANDLER_H

#include "path_planner.h"
#include "trajectory.h"
#include "car.h"

class Handler
{
public:
    Handler(Trajectory *trajectory);

    void HandlePathPlanning(const MapWaypoints & map_waypoints,
                            CarData & car,
                            vector<double> & next_x_vals,
                            vector<double> & next_y_vals);

private:
    PathPlanner * path_planner;
    Trajectory * trajectory;
};


#endif //PATH_PLANNING_HANDLER_H
