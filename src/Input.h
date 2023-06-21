//
// Created by MacBook Pro on 02/06/2022.
//

#ifndef DOOR2DOOR2_INPUT_H
#define DOOR2DOOR2_INPUT_H


#include <vector>
#include "string"
#include "Config.h"

class Input {
public:
    std::vector<std::vector<double>> coordinates;
    std::vector<std::vector<double>> distances;
    std::vector<std::vector<double>> droneTimes;
    std::vector<std::vector<double>> techTimes;
    std::vector<double> demand ;
    int numCus{};
    std::string dataSet;
    std::vector<bool> cusOnlyServedByTech;

    Input(double droneVelocity, double techVelocity, int limitationFightTime, const std::string& path);

    Input();
};


#endif //DOOR2DOOR2_INPUT_H
