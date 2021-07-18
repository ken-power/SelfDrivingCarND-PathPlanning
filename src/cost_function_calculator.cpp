//
// Created by Ken Power on 18/07/2021.
//
#include "cost_function_calculator.h"

vector<double> & CostFunctionCalculator::GetScoresForLane(vector<double> & scores,
                                                          int lane_number,
                                                          const vector<double> & front_vehicle,
                                                          const vector<double> & back_vehicle)
{
    if(front_vehicle[0] > 1000 and back_vehicle[0] > 1000)
    {
        scores[lane_number] += 5; // if wide open lane, move into that lane
    }
    else
    {
        if(front_vehicle[0] < 10)
        {
            scores[lane_number] -= 5; // if car too close in front, negative score
        }
        if(back_vehicle[0] < 10)
        {
            scores[lane_number] -= 5; // if car too close in back, negative score
        }

        scores[lane_number] += 1 - (10 / (front_vehicle[0] / 3)); // benefit for large open distance in lane in front
        scores[lane_number] += 1 - (10 / (back_vehicle[0] / 3)); // benefit for large open distance in lane in back
        scores[lane_number] += 1 - (10 / (front_vehicle[1] / 2)); // benefit for faster car speed in lane in front
        scores[lane_number] += 1 / (back_vehicle[1] / 2); // benefit for slower car speed in lane in back
    }
    return scores;
}

void CostFunctionCalculator::UpdateAverageLaneScores(const vector<double> & scores,
                                                     int lane_number,
                                                     vector<double> & lane_scores)
{
    const int iterations = 10;  // simple calculation based on 10 iterations
    lane_scores[lane_number] = (lane_scores[lane_number] * iterations) - lane_scores[lane_number];
    lane_scores[lane_number] += scores[lane_number];
    lane_scores[lane_number] /= iterations;
}

const double CostFunctionCalculator::KeepCurrentLane()
{
    // benefit to keeping lane
    return 0.5;
}