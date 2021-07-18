//
// Created by Ken Power on 16/07/2021.
//
#include<cmath>
#include "path_planner.h"
#include "cost_function_calculator.h"

int PathPlanner::MoveDistance(double s, double d, const vector<vector<double>> & sensor_fusion)
{
    int lane = GetLane(d);
    int new_lane;
    double distance = NearestVehicle(s, lane, sensor_fusion, true)[0];

    current_lane = lane; // Keep the current lane to later calculate desired move

    const int min_threshold_distance = 20;
    // check if blocked, i.e. car is within min_threshold_distance of 20 meters

    if(distance > min_threshold_distance)
    { // if lots of space, stay in lane and go near the speed limit
        new_lane = lane;
        target_vehicle_speed = 22.352 - 0.5;
        average_lane_scores = {0, 0, 0}; // Reset average scores for each lane
        return 0;
    }
    else
    {
        new_lane = ChooseLaneForNextStep(s, lane, sensor_fusion);
        vector<double> vehicle = NearestVehicle(s, new_lane, sensor_fusion, true);
        target_vehicle_speed = vehicle[1];
    }

    // Space between middle of each lane is four meters, so move accordingly
    const int lane_width = 4;  // Width between middle of each lane is 4 meters
    if(new_lane == lane)
    {
        return 0;
    }
    else if(new_lane < lane)
    {
        return -lane_width;
    }
    else
    {
        return lane_width;
    }
}

/**
 * Check which lane the d-value comes from. Assumes a 3-lane highway, where
 * 0 == left lane, 1 == middle lane, 2 == right lane.
 * @param d
 * @return the lane corresponding to the d-value
 */
int PathPlanner::GetLane(double d)
{
    int lane;
    if(d < 4)
    {
        lane = Left;
    }
    else if(d < 8)
    {
        lane = Middle;
    }
    else
    {
        lane = Right;
    }
    return lane;
}

vector<double> PathPlanner::NearestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction)
{
    double dist = 10000;
    double velocity = 22.352 - 0.5; // Set in case of no cars
    double vehicle_s;
    double vehicle_d;
    double vehicle_v;
    int vehicle_lane;

    // Check each vehicle in sensor range
    for(auto & vehicle : sensor_fusion)
    {
        vehicle_s = vehicle[5];
        vehicle_d = vehicle[6];
        vehicle_v = sqrt(pow(vehicle[3], 2) + pow(vehicle[4], 2));
        vehicle_lane = GetLane(vehicle_d);

        if(vehicle_lane == lane)
        { // if same lane
            if(direction)
            {
                if(vehicle_s > s and (vehicle_s - s) < dist)
                { // and ahead of my vehicle
                    dist = vehicle_s - s;
                    velocity = vehicle_v;
                }
            }
            else
            {
                if(s >= vehicle_s and (s - vehicle_s) < dist)
                { // if behind my vehicle
                    dist = s - vehicle_s;
                    velocity = vehicle_v;
                }
            }
        }
    }
    if(dist <= 0)
    { // Avoid dividing by zero in laneScore()
        dist = 1.0;
    }
    if(lane == current_lane and direction)
    {
        lead_vehicle_current_speed = velocity;
    }
    return {dist, velocity};
}

int PathPlanner::ChooseLaneForNextStep(double s, int current_lane, const vector<vector<double>> & sensor_fusion)
{
    vector<double> scores = {0, 0, 0};
    vector<double> front_vehicle;
    vector<double> back_vehicle;

    for(int lane_number = 0; lane_number < 3; lane_number++)
    {
        if(lane_number == current_lane)
        {
            scores[lane_number] += CostFunctionCalculator::KeepCurrentLane();
        }

        front_vehicle = NearestVehicle(s, lane_number, sensor_fusion, true);
        back_vehicle = NearestVehicle(s, lane_number, sensor_fusion, false);

        // Apply cost function calculations to help choose the lane for the next step in the path
        scores = CostFunctionCalculator::GetScoresForLane(scores, lane_number, front_vehicle, back_vehicle);
        CostFunctionCalculator::UpdateAverageLaneScores(scores, lane_number, average_lane_scores);
    }

    return GetNextLane(current_lane);
}


int PathPlanner::GetNextLane(int current_lane)
{
    if(current_lane == Left)  // left lane
    {
        return max_element(average_lane_scores.begin(), average_lane_scores.end() - 1) - average_lane_scores.begin();
    }
    else if(current_lane == Middle)  // middle lane
    {
        return max_element(average_lane_scores.begin(), average_lane_scores.end()) - average_lane_scores.begin();
    }
    else if(current_lane == Right) // right lane
    {
        return max_element(average_lane_scores.begin() + 1, average_lane_scores.end()) - average_lane_scores.begin();
    }
    else  // default to this if we get an invalid lane
    {
        return max_element(average_lane_scores.begin() + 1, average_lane_scores.end()) - average_lane_scores.begin();
    }
}


int PathPlanner::CurrentLane() const
{
    return current_lane;
}

double PathPlanner::LeadVehicleCurrentSpeed() const
{
    return lead_vehicle_current_speed;
}

double PathPlanner::TargetVehicleSpeed() const
{
    return target_vehicle_speed;
}

void PathPlanner::SetTargetVehicleSpeed(double targetVehicleSpeed)
{
    target_vehicle_speed = targetVehicleSpeed;
}

const vector<double> & PathPlanner::AverageLaneScores() const
{
    return average_lane_scores;
}

