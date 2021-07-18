#ifndef PATH_PLANNING_JSON_UTILS_H
#define PATH_PLANNING_JSON_UTILS_H

#include <cmath>
#include <string>
#include <vector>

#include "car.h"

using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string GetJsonData(const string& s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_first_of('}');

    if(found_null != string::npos)
    {
        return "";
    }
    else if(b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


/**
 * Get the car data from a JSON object
 *
 * @param j the JSON object - j[1] is the data JSON object
 * @return car data
 */
CarData GetCarData(auto &j)
{
    CarData car_data;

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

    return car_data;
}

#endif  // PATH_PLANNING_JSON_UTILS_H
