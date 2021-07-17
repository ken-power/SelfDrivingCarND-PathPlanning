//
// Created by Ken Power on 17/07/2021.
//

#ifndef PATH_PLANNING_HANDLER_H
#define PATH_PLANNING_HANDLER_H

#include "path_planner.h"
#include "trajectory.h"

class Handler
{
public:
    Handler(Trajectory *trajectory);

    void HandlePathPlanning(const vector<double> & map_waypoints_x,
                            const vector<double> & map_waypoints_y,
                            const vector<double> & map_waypoints_s,
                            double car_x,
                            double car_y,
                            double car_s,
                            double car_yaw,
                            double car_speed,
                            auto previous_path_x,
                            auto previous_path_y,
                            auto sensor_fusion,
                            vector<double> & next_x_vals,
                            vector<double> & next_y_vals);

private:
    PathPlanner * path_planner;
    Trajectory * trajectory;
};


#endif //PATH_PLANNING_HANDLER_H
