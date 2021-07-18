//
// Created by Ken Power on 16/07/2021.
//

#ifndef PATH_PLANNING_PATH_PLANNER_H
#define PATH_PLANNING_PATH_PLANNER_H

#include <vector>
#include <string>
#include "cost_function_calculator.h"

using namespace std;

enum Lane {Left, Middle, Right};

class PathPlanner
{
public:
    /**
     * Plan a path by deciding whether to move left, right, or stay in the same lane. A negative number indicates the
     * distance in meters to move left, a positive number indicates the distance in meters to move right, 0 means keep
     * moving forward.
     *
     * @param s
     * @param d
     * @param sensor_fusion
     * @return distance in meters to move (left, right, or stay on same trajectory)
     */
    int MoveDistance(double s, double d, const vector<vector<double>> & sensor_fusion);

    /**
     * Calculates if d value corresponds to left, right, or center lane.
     *
     * @param d
     * @return
     */
    int GetLane(double d);

    /**
     * Calculates the nearest vehicle either in front or behind the car in a given lane.
     *
     * @param s
     * @param lane
     * @param sensor_fusion
     * @param direction
     * @return distance and speed of the nearest vehicle
     */
    vector<double> NearestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);

    /**
     * Decide which lane to be in for the next step of the path.
     *
     * @param s s value
     * @param current_lane the lane the car is in now
     * @param sensor_fusion sensor fusion data
     * @return the lane to be in for the next step in the path, which might be the same as the current lane
     */
    int ChooseLaneForNextStep(double s, int current_lane, const vector<vector<double>> & sensor_fusion);

    int CurrentLane() const;

    double LeadVehicleCurrentSpeed() const;

    double TargetVehicleSpeed() const;

    void SetTargetVehicleSpeed(double targetVehicleSpeed);

    /**
     * @return the current average lane scores
     */
    const vector<double> & AverageLaneScores() const;

private:
    int current_lane;
    double lead_vehicle_current_speed = 22.352 - 0.5;
    double target_vehicle_speed;
    vector<double> average_lane_scores = {0, 0, 0};  // Assumes a 3-lane road
    int GetNextLane(int current_lane);
};

#endif //PATH_PLANNING_PATH_PLANNER_H
