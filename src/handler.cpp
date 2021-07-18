//
// Created by Ken Power on 17/07/2021.
//

#include "Handler.h"
#include "coordinate_transforms.h"
#include "waypoints.h"

Handler::Handler(Trajectory *trajectory)
{
    this->trajectory = trajectory;
    this->path_planner = trajectory->GetPathPlanner();
}


void Handler::HandlePathPlanning(const MapWaypoints & map_waypoints,
                                 CarData & car,
                                 vector<double> & next_x_vals,
                                 vector<double> & next_y_vals)
{
    int previous_path_size = car.previous_path.x.size();

    // Start with remaining old path
    for(int i = 0; i < previous_path_size; i++)
    {
        next_x_vals.push_back(car.previous_path.x[i]);
        next_y_vals.push_back(car.previous_path.y[i]);
    }

    // create a list of widely-spaced (x,y) waypoints, evenly spaced at 30m
    // later we will interpolate these waypoints with a spline and fill it in with more points that control speed
    vector<double> ptsx;
    vector<double> ptsy;

    // reference x,y,yaw states
    // either we will reference the starting point as where the car is, or at the previous path's end point
    double ref_x = car.localization.x;
    double ref_y = car.localization.y;
    double ref_yaw = Degrees2Radians(car.localization.yaw);
    double reference_velocity = 0.0;

    this->trajectory->DetermineStartingReference(car,
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
                                                    map_waypoints.x,
                                                    map_waypoints.y);

    double move = this->path_planner->MoveDistance(frenet_coords[0], frenet_coords[1], car.sensor_fusion);
    double lane = this->path_planner->CurrentLane();
    double next_d = (lane * 4) + 2 + move;

    // Double-check that the car has not incorrectly chose a blocked lane
    int check_lane = this->path_planner->GetLane(next_d);
    vector<double> front_vehicle = this->path_planner->NearestVehicle(frenet_coords[0],
                                                                      check_lane,
                                                                      car.sensor_fusion,
                                                                      true);
    vector<double> back_vehicle = this->path_planner->NearestVehicle(frenet_coords[0],
                                                                     check_lane,
                                                                     car.sensor_fusion,
                                                                     false);

    // Reset to current lane and leading vehicle if not enough room
    if(front_vehicle[0] < 10 or back_vehicle[0] < 10 or this->path_planner->AverageLaneScores()[check_lane] <= -5)
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
    this->trajectory->SetWaypoints(map_waypoints, car.localization.s, next_d, wp1, wp2, wp3);

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

