//
// Created by Ken Power on 18/07/2021.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

struct CarData
{
    struct Localization
    {
        // TODO remove car_ prefix in the names
        double car_x = 0.0;
        double car_y = 0.0;
        double car_s = 0.0;
        double car_d = 0.0;
        double car_yaw = 0.0;
        double car_speed = 0.0;
    } localization;

    struct PreviousPath
    {
        // Previous path data given to the Planner
        vector<double> previous_path_x;
        vector<double>  previous_path_y;

        // Previous path's end s and d values
        double end_path_s = 0.0;
        double end_path_d = 0.0;
    } previous_path;

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    vector<vector<double>> sensor_fusion;
};



#endif //PATH_PLANNING_CAR_H
