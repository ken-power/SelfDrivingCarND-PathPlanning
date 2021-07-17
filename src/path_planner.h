//
// Created by Ken Power on 16/07/2021.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <vector>
#include <string>

using namespace std;

class PathPlanner
{
public:
    // Decides whether to go left, right, or stay in the same lane
    // Returns amount of meters left or right to move
    int PlanPath(double s, double d, const vector<vector<double>> & sensor_fusion);

    // Calculates if d value corresponds to left, right, or center lane
    int GetLane(double d);

    // Calculates the nearest vehicle either in front or behind the car in a given lane
    // Returns distance and speed of that vehicle
    vector<double> NearestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);

    // Scores each lane on factors such as distance to nearest vehicle & speed
    // Returns the lane with the best score (0 left, 1 middle, 2 right)
    int BestLane(double s, int lane, vector<vector<double>> sensor_fusion);

    int CurrentLane() const;

    double LeadVehicleCurrentSpeed() const;

    double TargetVehicleSpeed() const;

    void SetTargetVehicleSpeed(double targetVehicleSpeed);

    const vector<double> & AverageScores() const;

private:
    int current_lane;
    double lead_vehicle_current_speed = 22.352 - 0.5;
    double target_vehicle_speed;
    vector<double> average_scores = {0, 0, 0};
};


#endif //PATH_PLANNING_PATH_PLANNER_H
