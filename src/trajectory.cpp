//
// Created by Ken Power on 17/07/2021.
//

#include "trajectory.h"
#include "spline.h"
#include "coordinate_transforms.h"

Trajectory::Trajectory(PathPlanner *path_planner)
{
    this->path_planner = path_planner;
}

PathPlanner *Trajectory::GetPathPlanner()
{
    return this->path_planner;
}

void Trajectory::DetermineStartingReference(CarData & car,
                                            int previous_path_size,
                                            vector<double> & ptsx,
                                            vector<double> & ptsy,
                                            double & ref_x,
                                            double & ref_y,
                                            double & ref_yaw,
                                            double & reference_velocity)
{
    // if previous size is almost empty, use the car as starting reference
    if(previous_path_size < 2)
    {
        // use two points that make the path tangent to the car
        double prev_car_x = car.localization.x - cos(car.localization.yaw);
        double prev_car_y = car.localization.y - sin(car.localization.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car.localization.x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car.localization.y);

        reference_velocity = car.localization.speed;
    }
    else  // use the previous path's end point as starting reference
    {
        // Redefine reference state as previous path end point
        ref_x = car.previous_path.x[previous_path_size - 1];
        ref_y = car.previous_path.y[previous_path_size - 1];

        double ref_x_prev = car.previous_path.x[previous_path_size - 2];
        double ref_y_prev = car.previous_path.y[previous_path_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
        reference_velocity = this->path_planner->TargetVehicleSpeed();

        // Use two points that make the path tangent to the previous path's end point
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }
}


void Trajectory::SetWaypoints(const MapWaypoints & map_waypoints,
                              double car_s,
                              double next_d,
                              vector<double> & wp1,
                              vector<double> & wp2,
                              vector<double> & wp3)
{
    // In Frenet add evenly-spaced points 50m apart ahead of the starting reference
    const int frenet_distance_spacing = 50;  // in meters

    wp1 = Frenet2Cartesian(car_s + frenet_distance_spacing,
                           next_d,
                           map_waypoints.s,
                           map_waypoints.x,
                           map_waypoints.y);
    wp2 = Frenet2Cartesian(car_s + (frenet_distance_spacing * 2),
                           next_d,
                           map_waypoints.s,
                           map_waypoints.x,
                           map_waypoints.y);
    wp3 = Frenet2Cartesian(car_s + (frenet_distance_spacing * 3),
                           next_d,
                           map_waypoints.s,
                           map_waypoints.x,
                           map_waypoints.y);// Set further waypoints based on going further along highway in desired lane
}


double Trajectory::CalculateTrajectory(vector<double> & next_x_vals,
                                       vector<double> & next_y_vals,
                                       int previous_path_size,
                                       vector<double> & ptsx,
                                       vector<double> & ptsy,
                                       double ref_x,
                                       double ref_y,
                                       double ref_yaw,
                                       double reference_velocity)
{
    if(ptsx.size() > 2)
    {  // Spline fails if not greater than two points - Otherwise just use rest of old path
        // Shift and rotate points to local coordinates
        for(int i = 0; i < ptsx.size(); i++)
        {
            // shift car reference angle to 0 degrees
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
        }

        // create a spline
        tk::spline spline;

        // set (x,y) points to the spline
        spline.set_points(ptsx, ptsy);

        // Calculate how to break up spline points so that we travel at our desired reference velocity
        const int horizon = 30; // our horizon, going out 30 meters
        double target_x = horizon;
        double target_y = spline(target_x); // the spline fives us the y value corresponding to x
        double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));  // distance calculation from car to target

        double x_add_on = 0;  // start at the origin
        const int max_acceleration = 10; // m/s/s
        const double acceleration = max_acceleration * 0.02 * 0.8; // Limit acceleration to acceptable range

        // Fill up the rest of our path planner after filling it with previous points, i.e., add on points
        // that are along the spline Here, we will always output 50 points
        for(int i = 0; i < 50 - previous_path_size; i++)
        {
            if(reference_velocity < this->path_planner->TargetVehicleSpeed() - acceleration)
            {
                // Accelerate if under target speed
                reference_velocity += acceleration;
            }
            else if(reference_velocity > this->path_planner->TargetVehicleSpeed() + acceleration)
            {
                // Brake if below target
                reference_velocity -= acceleration;
            }

            // Calculate points along new path
            double N = (target_dist / (.02 * reference_velocity));
            double x_point = x_add_on + (target_x) / N;
            double y_point = spline(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }
    }
    return reference_velocity;
}
