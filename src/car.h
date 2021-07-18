//
// Created by Ken Power on 18/07/2021.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>

using std::vector;

struct CarData
{
    struct Localization
    {
        double x = 0.0;
        double y = 0.0;
        double s = 0.0;
        double d = 0.0;
        double yaw = 0.0;
        double speed = 0.0;
    } localization;

    struct PreviousPath
    {
        // Previous path data given to the Planner
        vector<double> x;
        vector<double> y;

        // Previous path's end s and d values
        double s = 0.0;
        double d = 0.0;
    } previous_path;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    vector<vector<double>> sensor_fusion;
};



#endif //PATH_PLANNING_CAR_H
