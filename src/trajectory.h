//
// Created by Ken Power on 17/07/2021.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include "path_planner.h"

class Trajectory
{
public:
    Trajectory(PathPlanner *path_planner);

    void DetermineStartingReference(double car_x,
                                    double car_y,
                                    double car_yaw,
                                    double car_speed,
                                    auto previous_path_x,
                                    auto previous_path_y,
                                    int previous_path_size,
                                    vector<double> & ptsx,
                                    vector<double> & ptsy,
                                    double & ref_x,
                                    double & ref_y,
                                    double & ref_yaw,
                                    double & reference_velocity);

    void SetWaypoints(const vector<double> & map_waypoints_x,
                      const vector<double> & map_waypoints_y,
                      const vector<double> & map_waypoints_s,
                      double car_s,
                      double next_d,
                      vector<double> & wp1,
                      vector<double> & wp2,
                      vector<double> & wp3);

    double CalculateTrajectory(vector<double> & next_x_vals,
                               vector<double> & next_y_vals,
                               int previous_path_size,
                               vector<double> & ptsx,
                               vector<double> & ptsy,
                               double ref_x,
                               double ref_y,
                               double ref_yaw,
                               double reference_velocity);

private:
    PathPlanner * path_planner;
};


#endif //PATH_PLANNING_TRAJECTORY_H
