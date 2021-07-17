//
// Created by Ken Power on 17/07/2021.
//

#include "Handler.h"
#include "coordinate_transforms.h"

Handler::Handler(Trajectory *trajectory)
{
    this->trajectory = trajectory;
    this->path_planner = trajectory->GetPathPlanner();
}


void Handler::HandlePathPlanning(const vector<double> & map_waypoints_x,
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
                                 vector<double> & next_y_vals)
{
    int previous_path_size = previous_path_x.size();

    // Start with remaining old path
    for(int i = 0; i < previous_path_size; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // create a list of widely-spaced (x,y) waypoints, evenly spaced at 30m
    // later we will interpolate these waypoints with a spline and fill it in with more points that control speed
    vector<double> ptsx;
    vector<double> ptsy;

    // reference x,y,yaw states
    // either we will reference the starting point as where the car is, or at the previous path's end point
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = Degrees2Radians(car_yaw);
    double reference_velocity = 0.0;

    this->trajectory->DetermineStartingReference(car_x,
                                                 car_y,
                                                 car_yaw,
                                                 car_speed,
                                                 previous_path_x,
                                                 previous_path_y,
                                                 previous_path_size,
                                                 ptsx,
                                                 ptsy,
                                                 ref_x,
                                                 ref_y,
                                                 ref_yaw,
                                                 reference_velocity);

    // Plan the rest of the path based on calculations
    vector<double> frenet_coords = Cartesian2Frenet(ref_x,
                                                    ref_y,
                                                    ref_yaw,
                                                    map_waypoints_x,
                                                    map_waypoints_y);

    double move = this->path_planner->PlanPath(frenet_coords[0], frenet_coords[1], sensor_fusion);
    double lane = this->path_planner->CurrentLane();
    double next_d = (lane * 4) + 2 + move;

    // Double-check that the car has not incorrectly chose a blocked lane
    int check_lane = this->path_planner->GetLane(next_d);
    vector<double> front_vehicle = this->path_planner->NearestVehicle(frenet_coords[0],
                                                                      check_lane,
                                                                      sensor_fusion,
                                                                      true);
    vector<double> back_vehicle = this->path_planner->NearestVehicle(frenet_coords[0],
                                                                     check_lane,
                                                                     sensor_fusion,
                                                                     false);

    // Reset to current lane and leading vehicle if not enough room
    if(front_vehicle[0] < 10 or back_vehicle[0] < 10 or this->path_planner->AverageScores()[check_lane] <= -5)
    {
        next_d = (lane * 4) + 2;
        if(check_lane != lane)
        {
            this->path_planner->SetTargetVehicleSpeed(this->path_planner->LeadVehicleCurrentSpeed());
        }
    }

    vector<double> wp1;
    vector<double> wp2;
    vector<double> wp3;
    this->trajectory->SetWaypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s, car_s, next_d, wp1, wp2, wp3);

    ptsx.push_back(wp1[0]);
    ptsx.push_back(wp2[0]);
    ptsx.push_back(wp3[0]);

    ptsy.push_back(wp1[1]);
    ptsy.push_back(wp2[1]);
    ptsy.push_back(wp3[1]);

    reference_velocity = this->trajectory->CalculateTrajectory(next_x_vals,
                                                               next_y_vals,
                                                               previous_path_size,
                                                               ptsx,
                                                               ptsy,
                                                               ref_x,
                                                               ref_y,
                                                               ref_yaw,
                                                               reference_velocity);

    // Save the end speed to be used for the next path
    this->path_planner->SetTargetVehicleSpeed(reference_velocity);
}

