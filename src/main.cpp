#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "network_utils.h"
#include "json.hpp"
#include "path_planner.cpp"
#include "coordinate_transforms.h"
#include "distance_utils.h"
#include "trajectory.cpp"

using nlohmann::json;
using std::string;
using std::vector;

PathPlanner path_planner;
Trajectory trajectory = Trajectory(&path_planner);

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

        if(!s.empty())
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

                HandlePathPlanning(map_waypoints_x,
                                   map_waypoints_y,
                                   map_waypoints_s,
                                   car_x,
                                   car_y,
                                   car_s,
                                   car_yaw,
                                   car_speed,
                                   previous_path_x,
                                   previous_path_y,
                                   sensor_fusion,
                                   next_x_vals,
                                   next_y_vals);


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

    trajectory.DetermineStartingReference(car_x,
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

    vector<double> wp1;
    vector<double> wp2;
    vector<double> wp3;
    trajectory.SetWaypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s, car_s, next_d, wp1, wp2, wp3);

    ptsx.push_back(wp1[0]);
    ptsx.push_back(wp2[0]);
    ptsx.push_back(wp3[0]);

    ptsy.push_back(wp1[1]);
    ptsy.push_back(wp2[1]);
    ptsy.push_back(wp3[1]);

    reference_velocity = trajectory.CalculateTrajectory(next_x_vals,
                                                        next_y_vals,
                                                        previous_path_size,
                                                        ptsx,
                                                        ptsy,
                                                        ref_x,
                                                        ref_y,
                                                        ref_yaw,
                                                        reference_velocity);

    // Save the end speed to be used for the next path
    path_planner.SetTargetVehicleSpeed(reference_velocity);
}

