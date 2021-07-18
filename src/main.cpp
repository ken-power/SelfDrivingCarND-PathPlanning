#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "network_utils.h"
#include "json.hpp"

#include "path_planner.cpp"
#include "trajectory.cpp"
#include "handler.cpp"
#include "waypoints.h"
#include "car.h"

using nlohmann::json;
using std::string;
using std::vector;

void ParseJsonTelemetryData(CarData & car_data, auto &j);

int main()
{
    PathPlanner path_planner;
    Trajectory trajectory = Trajectory(&path_planner);
    Handler handler = Handler(&trajectory);

    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    WaypointData waypoint_data;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    // Each line in the file contains one set of waypoint data. Add them all to our set of waypoint data.
    string line;
    while(getline(in_map_, line))
    {
        PopulateWaypointsData(waypoint_data, line);
    }

    h.onMessage([&handler, &waypoint_data]
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
                CarData car_data;  // data for the main car
                ParseJsonTelemetryData(car_data, j);

                // ------------------------------------------------
                // -------- START OF PROJECT-SPECIFIC CODE --------

                // Define the actual (x,y) points we will use for the planner
                vector<double> next_x_vals;
                vector<double> next_y_vals;

                handler.HandlePathPlanning(waypoint_data,
                                           car_data,
                                           next_x_vals,
                                           next_y_vals);


                // -------- END OF PROJECT-SPECIFIC CODE --------
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


void ParseJsonTelemetryData(CarData & car_data, auto &j)
{
    // j[1] is the data JSON object

    // Main car's localization Data
    car_data.localization.x = j[1]["x"];
    car_data.localization.y = j[1]["y"];
    car_data.localization.s = j[1]["s"];
    car_data.localization.d = j[1]["d"];
    car_data.localization.yaw = j[1]["yaw"];
    car_data.localization.speed = j[1]["speed"];

    // Previous path data given to the Planner
    auto previous_path_x = j[1]["previous_path_x"];
    auto previous_path_y = j[1]["previous_path_y"];

    for(auto item : previous_path_x)
    {
        car_data.previous_path.x.push_back(item);
    }

    for(auto item : previous_path_y)
    {
        car_data.previous_path.y.push_back(item);
    }

    // Previous path's end s and d values
    car_data.previous_path.s = j[1]["end_path_s"];
    car_data.previous_path.d = j[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = j[1]["sensor_fusion"];

    for(auto item : sensor_fusion)
    {
        car_data.sensor_fusion.push_back(item);
    }
}
