#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "json_utils.h"
#include "json.hpp"

#include "path_planner.cpp"
#include "trajectory.cpp"
#include "handler.cpp"
#include "waypoints.h"
#include "car.h"

using nlohmann::json;
using std::string;
using std::vector;


int main()
{
    PathPlanner path_planner;
    Trajectory trajectory = Trajectory(&path_planner);
    Handler handler = Handler(&trajectory);
    MapWaypoints map_waypoints;  // map values for waypoint's x,y,s and d normalized normal vectors

    uWS::Hub h;

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    // Each line in the file contains one set of waypoint data. Add them all to our set of waypoint data.
    string line;
    while(getline(in_map_, line))
    {
        PopulateWaypointsData(map_waypoints, line);
    }

    h.onMessage([&handler, &map_waypoints]
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
                // data for the main car
                CarData car_data = GetCarData(j);

                // Define the actual (x,y) points we will use for the planner
                vector<double> next_x_vals;
                vector<double> next_y_vals;

                handler.HandlePathPlanning(map_waypoints,
                                           car_data,
                                           next_x_vals,
                                           next_y_vals);

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
