//
// Created by Ken Power on 16/07/2021.
//
#include<cmath>
#include "path_planner.h"

int PathPlanner::PlanPath(double s, double d, const vector<vector<double>>& sensor_fusion)
{
    int lane = GetLane(d);
    int new_lane;
    double distance = NearestVehicle(s, lane, sensor_fusion, true)[0];

    current_lane = lane; // Keep the current lane to later calculate desired move

    // check if blocked, i.e. car is within 20 meters
    if(distance > 20)
    { // if lots of space, stay in lane and go near the speed limit
        new_lane = lane;
        target_vehicle_speed = 22.352 - 0.5;
        average_scores = {0, 0, 0}; // Reset average scores for laneScore()
        return 0;
    }
    else
    {
        new_lane = BestLane(s, lane, sensor_fusion);
        vector<double> vehicle = NearestVehicle(s, new_lane, sensor_fusion, true);
        target_vehicle_speed = vehicle[1];
    }

    // Space between middle of each lane is four meters, so move accordingly
    if(new_lane == lane)
    {
        return 0;
    }
    else if(new_lane < lane)
    {
        return -4;
    }
    else
    {
        return 4;
    }
}

int PathPlanner::GetLane(double d)
{
    // Check which lane the d-value comes from
    // Left is 0, middle is 1, right is 2
    int lane;
    if(d < 4)
    {
        lane = 0;
    }
    else if(d < 8)
    {
        lane = 1;
    }
    else
    {
        lane = 2;
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

int PathPlanner::BestLane(double s, int lane, vector<vector<double>> sensor_fusion)
{
    vector<double> scores = {0, 0, 0};
    vector<double> front_vehicle;
    vector<double> back_vehicle;

    for(int i = 0; i < 3; i++)
    {
        if(i == lane)
        {  // benefit to keeping lane
            scores[i] += 0.5;
        }
        front_vehicle = NearestVehicle(s, i, sensor_fusion, true);
        back_vehicle = NearestVehicle(s, i, sensor_fusion, false);
        if(front_vehicle[0] > 1000 and back_vehicle[0] > 1000)
        {
            scores[i] += 5; // if wide open lane, move into that lane
        }
        else
        {
            if(front_vehicle[0] < 10)
            {
                scores[i] -= 5; // if car too close in front, negative score
            }
            if(back_vehicle[0] < 10)
            {
                scores[i] -= 5; // if car too close in back, negative score
            }
            scores[i] += 1 - (10 / (front_vehicle[0] / 3)); // benefit for large open distance in lane in front
            scores[i] += 1 - (10 / (back_vehicle[0] / 3)); // benefit for large open distance in lane in back
            scores[i] += 1 - (10 / (front_vehicle[1] / 2)); // benefit for faster car speed in lane in front
            scores[i] += 1 / (back_vehicle[1] / 2); // benefit for slower car speed in lane in back
        }

        // Simple in-exact calculation for scores over the last ten iterations
        average_scores[i] = (average_scores[i] * 10) - average_scores[i];
        average_scores[i] += scores[i];
        average_scores[i] /= 10;
    }

    // Only compare applicable lanes
    if(lane == 0)
    {
        return max_element(average_scores.begin(), average_scores.end() - 1) - average_scores.begin();
    }
    else if(lane == 1)
    {
        return max_element(average_scores.begin(), average_scores.end()) - average_scores.begin();
    }
    else
    {
        return max_element(average_scores.begin() + 1, average_scores.end()) - average_scores.begin();
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

const vector<double> & PathPlanner::AverageScores() const
{
    return average_scores;
}

