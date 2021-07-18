//
// Created by Ken Power on 18/07/2021.
//

#ifndef PATH_PLANNING_COST_FUNCTION_CALCULATOR_H
#define PATH_PLANNING_COST_FUNCTION_CALCULATOR_H

#include <vector>

using std::vector;

class CostFunctionCalculator
{
public:

    static vector<double> & GetScoresForLane(vector<double> & scores,
                                             int lane_number,
                                             const vector<double> & front_vehicle,
                                             const vector<double> & back_vehicle);

    static void UpdateAverageLaneScores(const vector<double> & scores, int lane_number, vector<double> & lane_scores);

    static const double KeepCurrentLane();

};


#endif //PATH_PLANNING_COST_FUNCTION_CALCULATOR_H
