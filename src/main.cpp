#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "network_utils.h"
#include "json.hpp"
#include "spline.h"
#include "path_planner.cpp"
#include "coordinate_transforms.h"
#include "distance_utils.h"

using nlohmann::json;
using std::string;
using std::vector;

PathPlanner path_planner;

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while(getline(in_map_, line))
    {
        std::istringstream iss(line);
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
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
                        &map_waypoints_dx, &map_waypoints_dy]
                        (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                         uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if(!length || length <= 2 || data[0] != '4' || data[1] != '2')
            return;
        auto s = GetJsonData(data);

        if(s != "")
        {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if(event == "telemetry")
            {
                // j[1] is the data JSON object

                // Main car's localization Data
                double car_x = j[1]["x"];
                double car_y = j[1]["y"];
                double car_s = j[1]["s"];
                double car_d = j[1]["d"];
                double car_yaw = j[1]["yaw"];
                double car_speed = j[1]["speed"];

                // Previous path data given to the Planner
                auto previous_path_x = j[1]["previous_path_x"];
                auto previous_path_y = j[1]["previous_path_y"];

                // Previous path's end s and d values
                double end_path_s = j[1]["end_path_s"];
                double end_path_d = j[1]["end_path_d"];

                // Sensor Fusion Data, a list of all other cars on the same side of the road.
                auto sensor_fusion = j[1]["sensor_fusion"];

                /**
                 * Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                 */
                // ------------------------------------------------
                // ------------------------------------------------
                // -------- START OF PROJECT-SPECIFIC CODE --------

                // Define the actual (x,y) points we will use for the planner
                vector<double> next_x_vals;
                vector<double> next_y_vals;


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

                // if previous size is almost empty, use the car as starting reference
                if(previous_path_size < 2)
                {
                    // use two points that make the path tangent to the car
                    double prev_car_x = car_x - cos(car_yaw);
                    double prev_car_y = car_y - sin(car_yaw);

                    ptsx.push_back(prev_car_x);
                    ptsx.push_back(car_x);

                    ptsy.push_back(prev_car_y);
                    ptsy.push_back(car_y);

                    reference_velocity = car_speed;
                }
                else  // use the previous path's end point as starting reference
                {
                    // Redefine reference state as previous path end point
                    ref_x = previous_path_x[previous_path_size - 1];
                    ref_y = previous_path_y[previous_path_size - 1];

                    double ref_x_prev = previous_path_x[previous_path_size - 2];
                    double ref_y_prev = previous_path_y[previous_path_size - 2];
                    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
                    reference_velocity = path_planner.TargetVehicleSpeed();

                    // Use two points that make the path tangent to the previous path's end point
                    ptsx.push_back(ref_x_prev);
                    ptsx.push_back(ref_x);

                    ptsy.push_back(ref_y_prev);
                    ptsy.push_back(ref_y);
                }

                // Plan the rest of the path based on calculations
                vector<double> frenet_coords = Cartesian2Frenet(ref_x,
                                                                ref_y,
                                                                ref_yaw,
                                                                map_waypoints_x,
                                                                map_waypoints_y);

                double move = path_planner.PlanPath(frenet_coords[0], frenet_coords[1], sensor_fusion);
                double lane = path_planner.CurrentLane();
                double next_d = (lane * 4) + 2 + move;

                // Double-check that the car has not incorrectly chose a blocked lane
                int check_lane = path_planner.GetLane(next_d);
                vector<double> front_vehicle = path_planner.NearestVehicle(frenet_coords[0],
                                                                           check_lane,
                                                                           sensor_fusion,
                                                                           true);
                vector<double> back_vehicle = path_planner.NearestVehicle(frenet_coords[0],
                                                                          check_lane,
                                                                          sensor_fusion,
                                                                          false);

                // Reset to current lane and leading vehicle if not enough room
                if(front_vehicle[0] < 10 or back_vehicle[0] < 10 or path_planner.AverageScores()[check_lane] <= -5)
                {
                    next_d = (lane * 4) + 2;
                    if(check_lane != lane)
                    {
                        path_planner.SetTargetVehicleSpeed(path_planner.LeadVehicleCurrentSpeed());
                    }
                }

                // Set further waypoints based on going further along highway in desired lane
                // In Frenet add evenly-spaced points 50m apart ahead of the starting reference
                vector<double> wp1 = Frenet2Cartesian(car_s + 50,
                                                      next_d,
                                                      map_waypoints_s,
                                                      map_waypoints_x,
                                                      map_waypoints_y);
                vector<double> wp2 = Frenet2Cartesian(car_s + 100,
                                                      next_d,
                                                      map_waypoints_s,
                                                      map_waypoints_x,
                                                      map_waypoints_y);
                vector<double> wp3 = Frenet2Cartesian(car_s + 150,
                                                      next_d,
                                                      map_waypoints_s,
                                                      map_waypoints_x,
                                                      map_waypoints_y);

                ptsx.push_back(wp1[0]);
                ptsx.push_back(wp2[0]);
                ptsx.push_back(wp3[0]);

                ptsy.push_back(wp1[1]);
                ptsy.push_back(wp2[1]);
                ptsy.push_back(wp3[1]);

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
                    double target_x = 30; // our horizon, going out 30
                    double target_y = spline(target_x); // the spline fives us the y value corresponding to x
                    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));  // distance calculation from car to target

                    double x_add_on = 0;  // start at the origin
                    const int MAX_ACCELERATION = 10; // m/s/s
                    const double acceleration = MAX_ACCELERATION * 0.02 * 0.8; // Limit acceleration to acceptable range

                    // Fill up the rest of our path planner after filling it with previous points, i.e., add on points
                    // that are along the spline Here, we will always output 50 points
                    for(int i = 0; i < 50 - previous_path_size; i++)
                    {
                        if(reference_velocity < path_planner.TargetVehicleSpeed() - acceleration)
                        {
                            // Accelerate if under target speed
                            reference_velocity += acceleration;
                        }
                        else if(reference_velocity > path_planner.TargetVehicleSpeed() + acceleration)
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

                // Save the end speed to be used for the next path
                path_planner.SetTargetVehicleSpeed(reference_velocity);

                // -------- END OF PROJECT-SPECIFIC CODE --------
                // ------------------------------------------------
                // ------------------------------------------------
                json msgJson;
                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;

                auto msg = "42[\"control\"," + msgJson.dump() + "]";

                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }  // end "telemetry" if
        }
        else
        {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
        // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if(h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }

    h.run();
}
