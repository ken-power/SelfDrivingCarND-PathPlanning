#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
void DriveInStraightLine(double dist_inc,
                         double car_x,
                         double car_y,
                         double car_yaw,
                         vector<double> & next_x_vals,
                         vector<double> & next_y_vals);

void DriveInCircles(double dist_inc,
                    double car_x,
                    double car_y,
                    double car_yaw,
                    auto& previous_path_x,
                    auto& previous_path_y,
                    vector<double> & next_x_vals,
                    vector<double> & next_y_vals);

using nlohmann::json;
using std::string;
using std::vector;

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
        if(length && length > 2 && data[0] == '4' && data[1] == '2')
        {

            auto s = hasData(data);

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

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
                     */
                     // -------- START OF PROJECT-SPECIFIC CODE --------

                    double distance_increment = 0.5;  // 0.5 meters is how much the points are spaced apart - turns out to be pretty close to the speed limit of 50mph
                    // DriveInStraightLine(distance_increment, car_x, car_y, car_yaw, next_x_vals, next_y_vals);
                    // DriveInCircles(distance_increment, car_x, car_y, car_yaw, previous_path_x, previous_path_y, next_x_vals, next_y_vals);
                    

                    // -------- END OF PROJECT-SPECIFIC CODE --------
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
        }  // end websocket if
    }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
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


void DriveInStraightLine(double dist_inc,
                         double car_x,
                         double car_y,
                         double car_yaw,
                         vector<double> & next_x_vals,
                         vector<double> & next_y_vals)
{
    for(int i = 0; i < 50; ++i)
    {
        next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
        next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
    }
}


void DriveInCircles(double dist_inc,
                    double car_x,
                    double car_y,
                    double car_yaw,
                    auto& previous_path_x,
                    auto& previous_path_y,
                    vector<double> & next_x_vals,
                    vector<double> & next_y_vals)
{
    double pos_x;
    double pos_y;
    double angle;
    int path_size = previous_path_x.size();

    for(int i = 0; i < path_size; ++i)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    if(path_size == 0)
    {
        pos_x = car_x;
        pos_y = car_y;
        angle = deg2rad(car_yaw);
    }
    else
    {
        pos_x = previous_path_x[path_size - 1];
        pos_y = previous_path_y[path_size - 1];

        double pos_x2 = previous_path_x[path_size - 2];
        double pos_y2 = previous_path_y[path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
    }

    for(int i = 0; i < 50 - path_size; ++i)
    {
        next_x_vals.push_back(pos_x + (dist_inc) * cos(angle + (i + 1) * (pi() / 100)));
        next_y_vals.push_back(pos_y + (dist_inc) * sin(angle + (i + 1) * (pi() / 100)));
        pos_x += (dist_inc) * cos(angle + (i + 1) * (pi() / 100));
        pos_y += (dist_inc) * sin(angle + (i + 1) * (pi() / 100));
    }
}

