//
// Created by MacBook Pro on 02/06/2022.
//

#include <fstream>
#include <sstream>
#include <iostream>
#include "Input.h"
#include <cmath>
#include <string>
#include <vector>
#include "nlohmann/json.hpp"

using namespace std;

Config config;

Input::Input(double droneVelocity, double techVelocity, int limitationFightTime, const std::string &path) {
    std::ifstream file(path);
    std::string line;
    std::string tmp;
    if (file.is_open()) {
        std::getline(file, line);
        std::getline(file, line);
        std::getline(file, line);
        std::getline(file, line);
        line.replace(line.begin(), line.begin()+ 10, "");

        line.erase(line.find_last_of(' '));

        numCus = std::stoi(line);
        std::getline(file, line);
        double x, y, z, a, b, c;

        coordinates.push_back({0, 0});
        demand.push_back(0);
        typeService.push_back(0);
        serviceTimeByTruck.push_back(0);
        serviceTimeByDrone.push_back(0);
        while (file >> x >> y >> z >> a >> b >> c) {
            coordinates.push_back({x, y});
            demand.push_back(z);
            typeService.push_back(a);
            serviceTimeByTruck.push_back(b);
            serviceTimeByDrone.push_back(c);
        }
        coordinates.push_back({0, 0});
        demand.push_back(0);
        typeService.push_back(0);
        serviceTimeByTruck.push_back(0);
        serviceTimeByDrone.push_back(0);
        for (int i = 0; i < numCus + 2; i++) {
            
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
            if ((demand[i] > config.droneCapacity) || (typeService[i] == 1)) {
                cusOnlyServedByTech[i] = true;
            }
            // std::cout << " Demand of cus " << i << " : " << demand[i] << " | " << cusOnlyServedByTech[i] << "\n";  
        }

        size_t beg = path.find_last_of("//");
        size_t end = path.find_last_of('.');

        dataSet = path.substr(0, end).substr(beg + 1);
    }
}


void Input::truck_Input(Input &input ,const std::string &truck_path){
    ifstream fJson(truck_path);
    stringstream buffer;
    buffer << fJson.rdbuf();
    auto json = nlohmann::json::parse(buffer.str());
    truckV_max = json["V_max (m/s)"];
    truckWeight_max = json["M_t (kg)"];
    auto time = json["T (hour)"];
    input.ratio_v.push_back ( time["0-1"] );
    input.ratio_v.push_back ( time["1-2"] );
    input.ratio_v.push_back ( time["2-3"] );
    input.ratio_v.push_back ( time["3-4"] );
    input.ratio_v.push_back ( time["4-5"] );
    input.ratio_v.push_back ( time["5-6"] );
    input.ratio_v.push_back ( time["6-7"] );
    input.ratio_v.push_back ( time["7-8"] );
    input.ratio_v.push_back ( time["8-9"] );
    input.ratio_v.push_back ( time["9-10"] );
    input.ratio_v.push_back ( time["10-11"] );
    input.ratio_v.push_back ( time["11-12"] );
}

Input::Input() = default;
