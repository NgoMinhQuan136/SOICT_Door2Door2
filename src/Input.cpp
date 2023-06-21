//
// Created by MacBook Pro on 02/06/2022.
//

#include <fstream>
#include <iostream>
#include "Input.h"
#include <cmath>

Input::Input(double droneVelocity, double techVelocity, int limitationFightTime, const std::string &path) {
    std::ifstream file(path);
    std::string line;
    std::string tmp;
    if (file.is_open()) {
        std::getline(file, line);
        line.replace(line.begin(), line.begin()+ 10, "");

        line.erase(line.find_last_of(' '));

        numCus = std::stoi(line);
        std::getline(file, line);
        double x, y, z;

        coordinates.push_back({0, 0});
        demand.push_back(0);
        while (file >> x >> y >> z) {
            coordinates.push_back({x, y});
            demand.push_back(z);
        }
        coordinates.push_back({0, 0});
        demand.push_back(0);

        for (int i = 0; i < numCus + 2; i++) {
            // std::cout << " Demand of cus " << i << " : " << demand[i] << "\n";  
            std::vector<double> iDistances;
            std::vector<double> iDroneTimes;
            std::vector<double> iTechTimes;
            for (int j = 0; j < numCus + 2; j++) {
                double distance = sqrt(pow(coordinates[i][0] - coordinates[j][0], 2) +
                                       pow(coordinates[i][1] - coordinates[j][1], 2));
                iDistances.push_back(distance);
                iDroneTimes.push_back(distance/droneVelocity);
                iTechTimes.push_back(distance/techVelocity);
            }

            distances.push_back(iDistances);
            droneTimes.push_back(iDroneTimes);
            techTimes.push_back(iTechTimes);
        }
        cusOnlyServedByTech.resize(numCus + 1, false);
        cusOnlyServedByTech[0] = false;

        for (int i = 1; i < numCus + 1; i++) {
            if (droneTimes[0][i] > limitationFightTime) {
                cusOnlyServedByTech[i] = true;
            }
        }

        size_t beg = path.find_last_of("//");
        size_t end = path.find_last_of('.');

        dataSet = path.substr(0, end).substr(beg + 1);
    }
}

Input::Input() = default;
