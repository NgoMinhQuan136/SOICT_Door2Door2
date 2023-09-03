//
// Created by MacBook Pro on 02/06/2022.
//

#include "Solution.h"
#include "Utils.h"
#include "map"
#include "nlohmann/json.hpp"
#include "Random.h"
#include <iomanip>


using json = nlohmann::json;
using Random = effolkronium::random_static;
Config config_1;
Input input;


double k1 = config_1.k1;
double k2 = config_1.k2;
double c1 = config_1.c1;
double c2 = config_1.c2;
double c4 = config_1.c4;
double c5 = config_1.c5;

double W = config_1.droneWeight; // drone weight; 
double g = config_1.g; // gravitational constant;
double alpha = config_1.alpha; // angle off attack;
double landing_speed = config_1.droneLandingSpeed;
double takeoff_speed = config_1.droneTakeoffSpeed;
double drone_speed = config_1.droneCruiseSpeed; //(m/s)
double h = config_1.cruiseAlt;
double drone_energy = config_1.droneBatteryPower;



double landing_Takeoff(double demand, double speed, double W){
    double P;
    P = k1*(W + demand)*g*(speed/2 + sqrt(pow((speed/2), 2.) + (W + demand)*g/pow(k2, 2.))) + c2*sqrt(pow(((W + demand)*g), 3.));
    return P;
}

double horizontal(double demand, double speed){
    double P;
    double P1 = pow(((W + demand)*g - c5*pow((speed*cos(alpha)), 2.)), 2.);
    double P2 = pow(c4*pow(speed, 2.), 2.);
    double P3 = c4*pow(speed, 3);
    double P4 = pow(speed, 3.);
    P = (c1 + c2)*pow((P1 + P2 ), 3/4.) + P3;
    return P;
}

double totalTripEnergy(std::vector<int> trip , Input input){
    double total_demand = 0;
    double time = 0;
    double total_Energy = landing_Takeoff(0, takeoff_speed, W)*h/takeoff_speed;
    // std::cout << "A : " << total_Energy << "\n";
    for (int i = 0; i < trip.size() - 1; i++){
        time = input.distances[trip[i]][trip[i + 1]] / drone_speed;
        total_demand += input.demand[trip[i]];
        total_Energy += horizontal(total_demand, drone_speed) * time ; 
    }
    total_Energy+= landing_Takeoff(total_demand, landing_speed, W)*h/landing_speed;
    
    return total_Energy;
}

int check_ratio(Input input, double time){
    int i;
    for (i = 0; i < input.ratio_v.size(); i++){
        if (i <= time && time < i + 1){
            break;
        }
    }
    return i;
}

double countTimeTruck(double startTime, double distance, Input input){ 
    double t = startTime / 3600;
    int i = check_ratio(input, startTime / 3600);
    double time;
    double round = 0;
    time = t + distance/(input.truckV_max*input.ratio_v[i] * 3600);
    while (time > i+1){
        if (time > 12){
            round++;
            time = time - 12;
        }
        distance = distance - input.truckV_max * input.ratio_v[i] * (i+1 - t) * 3600;
        t = i+1;
        time = t + distance/(input.truckV_max*input.ratio_v[i+1] * 3600);
        i++;
    }

    return ( time + round*12 - startTime / 3600) * 3600;
    
}

Solution *Solution::initSolution(Config &config, Input &input, InitType type, double alpha1, double alpha2) {
    Solution *best = nullptr, *current;
    double bestFeasibleScore = std::numeric_limits<double>::max(), currentScore;
    // if (type != DISTANCE) {
    //     current = new Solution(config, input, alpha1, alpha2);
    //     current->initByAngle(true, 1);
    //     currentScore = current->getScore()[0][0][0];
    //     if (currentScore < bestFeasibleScore) {
    //         best = current;
    //         bestFeasibleScore = currentScore;
    //     }
    //     current = new Solution(config, input, alpha1, alpha2);
    //     current->initByAngle(false, 1);
    //     currentScore = current->getScore()[0][0][0];
    //     if (currentScore < bestFeasibleScore) {
    //         best = current;
    //         bestFeasibleScore = currentScore;
    //     }
    //     current = new Solution(config, input, alpha1, alpha2);
    //     current->initByAngle(false, -1);
    //     currentScore = current->getScore()[0][0][0];
    //     if (currentScore < bestFeasibleScore) {
    //         best = current;
    //         bestFeasibleScore = currentScore;
    //     }
    //     current = new Solution(config, input, alpha1, alpha2);
    //     current->initByAngle(true, -1);
    //     currentScore = current->getScore()[0][0][0];
    //     if (currentScore < bestFeasibleScore) {
    //         best = current;
    //         bestFeasibleScore = currentScore;
    //     }
    // }
    if (type != ANGLE) {

        current = new Solution(config, input, alpha1, alpha2);
        double new_score = current->getScore()[0][0][0];
        current->initByDistance(false);
        currentScore = current->getScore()[0][0][0];
        if (currentScore < bestFeasibleScore) {
            best = current;
            bestFeasibleScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByDistance(true);
        currentScore = current->getScore()[0][0][0];
        if (currentScore < bestFeasibleScore) {
            best = current;
            bestFeasibleScore = currentScore;
        }
    }
    best->logConsole();
    return best;
    
}

void Solution::initByDistance(bool reverse) {
    std::vector<std::vector<double>> distances(input.distances);
    std::vector<std::vector<int>> orderDistance(input.numCus + 1);
    
    for (int i = 0; i < input.numCus + 1; i++) {
        for (size_t e: Utils::sortIndices(input.distances[i], reverse)) {
            if (e != i && e != input.numCus + 1) {
                orderDistance[i].emplace_back(e);
            }
        }
    }

    std::vector<double> travelTime(config.numTech + config.numDrone, 0);
    std::vector<bool> visitedCus(input.numCus + 1, false);
    std::vector<double> truckDemand(config.numTech, 0);
    visitedCus[0] = true;
    int numVisitedCus = 0;
    int nIter = 0, i = -1;
    while (numVisitedCus < input.numCus &&
           nIter < (config.numDrone + config.numTech) * input.numCus) {
        nIter++;
        i++;
        i %= config.numDrone + config.numTech;
        std::vector<std::vector<double>> time;
        int lastCus;
        int index = i;
        if (i < config.numDrone) {
            time = input.droneTimes;
            if (droneTripList[index].empty()) {
                droneTripList[index].emplace_back();
            }
            if (droneTripList[index].back().empty()) {
                lastCus = 0;
            } else {
                lastCus = droneTripList[index].back().back();
            }
            int nextCus = -1;
            for (int tmp: orderDistance[lastCus]) {
                if (!visitedCus[tmp] && !input.cusOnlyServedByTech[tmp]) {
                    nextCus = tmp;
                    break;
                }
            }

            if (nextCus <= 0) {
                continue;
            }
            double timeGoToFirstCus = time[0][nextCus] + input.serviceTimeByDrone[nextCus];
            if (!droneTripList[index].back().empty()) {
                timeGoToFirstCus = time[0][droneTripList[index].back()[0]] + input.serviceTimeByDrone[0];
            }

            double flightTime = travelTime[i] + time[lastCus][nextCus] + input.serviceTimeByDrone[nextCus] + time[nextCus][0];
            double waitTime = flightTime - timeGoToFirstCus;
            std::vector<int> test_trip(droneTripList[i].back());
            double droneTripDemand = 0;
            test_trip.push_back(nextCus);
            for (int k = 0; k < test_trip.size(); k++){
                droneTripDemand += input.demand[test_trip[k]];
            }
            test_trip.push_back(0);
            test_trip.insert(test_trip.begin(), 0);
            double energy = totalTripEnergy(test_trip, input);

            if ( energy > config.droneBatteryPower || waitTime > config.sampleLimitationWaitingTime || droneTripDemand > config.droneCapacity)  {
                if (!droneTripList[index].back().empty()){
                    droneTripList[index].emplace_back();
                }
                
                    
                // travelTime[i] = time[0][nextCus] + input.serviceTimeByDrone[nextCus];
            } else {
                travelTime[i] += time[lastCus][nextCus] + input.serviceTimeByDrone[nextCus];
                droneTripList[index].back().push_back(nextCus);
                visitedCus[nextCus] = true;
                numVisitedCus++;
            }
            
        } else {
            time = input.techTimes;
            index -= config.numDrone;
            if (techTripList[index].empty()) {
                techTripList[index].emplace_back();
            }
            if (techTripList[index].back().empty()) {
                lastCus = 0;
            } else {
                lastCus = techTripList[index].back().back();
            }
            int nextCus = -1;
            for (int tmp: orderDistance[lastCus]) {
                if (!visitedCus[tmp]) {
                    nextCus = tmp;
                    break;
                }
            }

            
            if (nextCus <= 0) {
                continue;
            }

            double timeGoToFirstCus = countTimeTruck(travelTime[i], distances[lastCus][nextCus], input) + input.serviceTimeByTruck[nextCus];
            if (!techTripList[index].back().empty()) {
                timeGoToFirstCus = countTimeTruck(travelTime[i], distances[0][techTripList[index].back()[0]], input);
            }
            
            double timeToNextCus = countTimeTruck(travelTime[i], distances[lastCus][nextCus], input) + input.serviceTimeByTruck[nextCus];
            double timeNextCusToDepot = countTimeTruck(travelTime[i]+timeToNextCus, distances[nextCus][0], input);
            double waitTime = travelTime[i] + timeToNextCus + input.serviceTimeByTruck[nextCus] + timeNextCusToDepot - timeGoToFirstCus;
            std::vector<int> test_trip(techTripList[index].back());
            double truckTripDemand = 0;
            test_trip.push_back(nextCus);
            for (int k = 0; k < test_trip.size(); k++){
                truckTripDemand += input.demand[test_trip[k]];
            }
            if (waitTime > config.sampleLimitationWaitingTime || truckTripDemand > input.truckWeight_max)  {
                techTripList[index].emplace_back();
                travelTime[i] = countTimeTruck(travelTime[i], distances[0][nextCus], input) + input.serviceTimeByDrone[nextCus];
            } else {
                travelTime[i] += timeToNextCus;
            }
            techTripList[index].back().push_back(nextCus);
            visitedCus[nextCus] = true;
            numVisitedCus++;
        }

        
    }
    
}

void Solution::initByAngle(bool reverse, int direction) {
    std::vector<std::vector<double>> distances(input.distances);
    std::vector<std::vector<int>> orderDistance(input.numCus + 1);
    std::vector<double> truckDemand(config.numTech, 0);

    for (int i = 0; i < input.numCus + 1; i++) {
        for (size_t e: Utils::sortIndices(input.distances[i], reverse)) {
            if (e != i && e != input.numCus + 1) {
                orderDistance[i].emplace_back(e);
            }
        }
    }
    int pivotCus = orderDistance[0][0];
    std::vector<std::vector<double>> coordinates = Utils::slice(input.coordinates, 1,
                                                                (int) input.coordinates.size() - 2);
    std::vector<double> angle;
    std::vector<double> perpendicular = {coordinates[pivotCus - 1][1], coordinates[pivotCus - 1][0] * -1};
    for (int i = 0; i < (int) coordinates.size(); i++) {
        if (i == pivotCus - 1) {
            angle.push_back(0.);
            continue;
        }
        double inner = std::inner_product(coordinates[pivotCus - 1].begin(),
                                          coordinates[pivotCus - 1].end(),
                                          coordinates[i].begin(),
                                          0.);
        double norm = sqrt(std::inner_product(coordinates[pivotCus - 1].begin(),
                                              coordinates[pivotCus - 1].end(),
                                              coordinates[pivotCus - 1].begin(),
                                              0.)) *
                      sqrt(std::inner_product(coordinates[i].begin(),
                                              coordinates[i].end(),
                                              coordinates[i].begin(),
                                              0.));
        double acos = inner / norm;
        double tmpAngle = std::acos(acos);
        double cc = std::inner_product(perpendicular.begin(), perpendicular.end(), coordinates[i].begin(), 0.);
        if ((direction == 1 && cc < 0) || direction == -1 && cc > 0) {
            tmpAngle = 2 * M_PI - tmpAngle;
        }
        angle.push_back(tmpAngle);
    }
    std::vector<int> orderAngle;
    for (size_t e: Utils::sortIndices(angle)) {
        orderAngle.push_back((int) e + 1);
    }
    std::vector<double> travelTime(config.numTech + config.numDrone, 0);
    std::vector<bool> visitedCus(input.numCus + 1, false);
    visitedCus[0] = true;
    int numVisitedCus = 0;
    int nIter = 0, i = 0;
    while (numVisitedCus < input.numCus &&
           nIter < (config.numDrone + config.numTech) * input.numCus) {
        nIter++;
        std::vector<std::vector<double>> time;
        int lastCus;
        int index = i;

        if (i < config.numDrone) {
            time = input.droneTimes;
            droneTripList[index].emplace_back();
            int j = 0;
            lastCus = 0;
            double timeGoToFirstCus = time[0][orderAngle[0]] + input.serviceTimeByDrone[orderAngle[0]];
            int remainCus = (int) orderAngle.size();
            while (j < remainCus) {
                int nextCus = orderAngle[j];
                if (!input.cusOnlyServedByTech[nextCus]) {
                    double flightTime = travelTime[i] + time[lastCus][nextCus] + input.serviceTimeByDrone[nextCus] + time[nextCus][0];
                    double waitTime = flightTime - timeGoToFirstCus;


                    std::vector<int> test_trip(droneTripList[i].back());
                    double droneTripDemand = 0;
                    test_trip.push_back(nextCus);
                    for (int k = 0; k < test_trip.size(); k++){
                        droneTripDemand += input.demand[test_trip[k]];
                        }
                    test_trip.push_back(0);
                    test_trip.insert(test_trip.begin(), 0);
                    double energy = totalTripEnergy(test_trip, input);
                    if ( waitTime <= config.sampleLimitationWaitingTime && energy < config.droneBatteryPower && droneTripDemand <= config.droneCapacity) {
                        travelTime[i] += time[lastCus][nextCus] + input.serviceTimeByDrone[nextCus];
                        droneTripList[index].back().push_back(nextCus);
                        visitedCus[nextCus] = true;
                        numVisitedCus++;
                        orderAngle.erase(orderAngle.begin() + j);
                        lastCus = nextCus;
                        remainCus--;
                    } else {
                        j++;
                    }
                } else {
                    j++;
                }
            }
        } else {
            index -= config.numDrone;
            techTripList[index].emplace_back();    
                time = input.techTimes;
                int j = 0;
                lastCus = 0;
                double timeGoToFirstCus = countTimeTruck(0, distances[0][orderAngle[0]], input) + input.serviceTimeByTruck[orderAngle[0]];
                int remainCus = (int) orderAngle.size();
                while (j < remainCus) {
                    int nextCus = orderAngle[j];
                    double timeToNextCus = countTimeTruck(travelTime[i], distances[lastCus][nextCus], input) + input.serviceTimeByTruck[nextCus];
                    double timeNextCusToDepot = countTimeTruck ( travelTime[i] + timeToNextCus, distances[nextCus][0], input);
                    double waitTime = travelTime[i] + timeToNextCus + timeNextCusToDepot - timeGoToFirstCus;

                    if (waitTime <= config.sampleLimitationWaitingTime && truckDemand[i] + input.demand[nextCus] < input.truckV_max) {
                        techTripList[index].back().push_back(nextCus);
                        travelTime[i] += timeToNextCus;
                        visitedCus[nextCus] = true;
                        numVisitedCus++;
                        orderAngle.erase(orderAngle.begin() + j);
                        lastCus = nextCus;
                        remainCus--;
                    } else {
                        j++;
                    }
                }
            
        }

        i++;
        i %= config.numDrone + config.numTech;
    }
}

std::vector<std::vector<std::vector<double>>> Solution::getScore() { 
    std::vector<double> techCompleteTime(config.numTech, 0);
    std::vector<double> droneCompleteTime(config.numDrone, 0);
    std::vector<double> cusCompleteTime(input.numCus + 1, 0);
    std::vector<std::vector<double>> droneTripCompleteTime;
    std::vector<std::vector<double>> truckTripCompleteTime;
    std::vector<std::vector<double>> totalDroneDemand;
    std::vector<std::vector<double>> totalTruckDemand;
    std::vector<std::vector<double>> overEnergyComplete;
    std::vector<std::vector<double>> droneWaitingTime;
    std::vector<std::vector<double>> truckWaitingTime;
    std::vector<std::vector<double>> distances(input.distances);
    std::vector<double> truckDemand(config.numTech, 0);
    int maxTrip = 0;
    for (auto &i: droneTripList) {
        if (maxTrip < i.size()) {
            maxTrip = (int) i.size();
        }
    }
    int maxTechTrip = 0;
    for (auto &i : techTripList){
        if (maxTechTrip < i.size()){
            maxTechTrip = (int) i.size();
        }
    }
    for (int i = 0; i < config.numDrone; i++) {
        std::vector<double> tripTime(maxTrip, 0);
        droneTripCompleteTime.push_back(tripTime);
    }
    for (int i = 0; i < config.numDrone; i++){
        std::vector<double> tripEnergy(maxTrip, 0);
        std::vector<double> tripDemand(maxTrip, 0);
        std::vector<double> tripWaitingTime(maxTrip, 0);
        overEnergyComplete.push_back(tripEnergy);
        totalDroneDemand.push_back(tripDemand);
        droneWaitingTime.push_back(tripWaitingTime);
    }
    for (int i = 0; i < config.numTech; i++){
        std::vector<double> tripTechTime(maxTechTrip, 0);
        std::vector<double> tripTechDemand(maxTechTrip, 0);
        std::vector<double> tripTechWaitingTime(maxTechTrip, 0);
        truckTripCompleteTime.push_back(tripTechTime);
        totalTruckDemand.push_back(tripTechDemand);
        truckWaitingTime.push_back(tripTechWaitingTime);
    }

    double tmp, tmp1;
    dz = 0, cz = 0, ez = 0;
    // cz: waiting time
    // dz: capacity truck and drone weight
    // ez : energy

    double allTechTime = 0, allDroneTime = 0;
    for (int i = 0; i < techTripList.size(); i++) {
        tmp = 0; tmp1 = 0;
        for(int j = 0; j < techTripList[i].size(); j++){
            if (techTripList[i][j].empty()) {
            continue;
            }
            totalTruckDemand[i][j] = input.demand[techTripList[i][j][0]];
            tmp1 = countTimeTruck(tmp1, distances[0][techTripList[i][j][0]], input) + input.serviceTimeByTruck[techTripList[i][j][0]];
            cusCompleteTime[techTripList[i][j][0]] = tmp1;
            for (int k = 0; k < (int) techTripList[i][j].size() - 1; k++) {
                tmp1 += countTimeTruck(tmp1, distances[techTripList[i][j][k]][techTripList[i][j][k + 1]], input) + input.serviceTimeByTruck[techTripList[i][j][k + 1]];
                cusCompleteTime[techTripList[i][j][k + 1]] = tmp1;
                totalTruckDemand[i][j] += input.demand[techTripList[i][j][k + 1]];
            }
            truckTripCompleteTime[i][j] = tmp1 + countTimeTruck(tmp , distances[techTripList[i][j].back()][0], input);
            tmp += truckTripCompleteTime[i][j];
        }
        techCompleteTime[i] = tmp;
        if (tmp > allTechTime){
            allTechTime = tmp;
        }
    }

    for (int i = 0; i < droneTripList.size(); i++) {
        tmp = 0;
        for (int j = 0; j < droneTripList[i].size(); j++) {
            if (droneTripList[i][j].empty()) {
                continue;
            }
            std::vector<int> eachTripDrone(droneTripList[i][j]);
            eachTripDrone.push_back(0);
            eachTripDrone.insert(eachTripDrone.begin(), 0);
            overEnergyComplete[i][j] = totalTripEnergy(eachTripDrone, input) ;
            totalDroneDemand[i][j] = input.demand[droneTripList[i][j][0]];
            
            tmp1 = input.droneTimes[0][droneTripList[i][j][0]] + input.serviceTimeByDrone[droneTripList[i][j][0]];
            cusCompleteTime[droneTripList[i][j][0]] = tmp1;

            for (int k = 0; k < (int) droneTripList[i][j].size() - 1; k++) {
                tmp1 += (input.droneTimes[droneTripList[i][j][k]][droneTripList[i][j][k + 1]] + input.serviceTimeByDrone[droneTripList[i][j][k + 1]]);
                cusCompleteTime[droneTripList[i][j][k + 1]] = tmp1;
                totalDroneDemand[i][j] += input.demand[droneTripList[i][j][k+1]];
            }
            droneTripCompleteTime[i][j] = tmp1 + input.droneTimes[droneTripList[i][j].back()][0];
            tmp += droneTripCompleteTime[i][j];
        }
        droneCompleteTime[i] = tmp;
        if (tmp > allDroneTime) {
            allDroneTime = tmp;
        }
    }

    c = std::max(allDroneTime, allTechTime);

    for(int i = 0; i < techTripList.size(); i++){
        for(int j = 0; j < techTripList[i].size(); j++){
            if(techTripList[i][j].empty()){
                continue;
            }
            for(int k: techTripList[i][j]){
                cz += std::max(0., truckTripCompleteTime[i][j] - cusCompleteTime[k] - config.sampleLimitationWaitingTime);
                
            }
            truckWaitingTime[i][j] = truckTripCompleteTime[i][j] - cusCompleteTime[techTripList[i][j][0]];
            double overWeightTruck = totalTruckDemand[i][j] - input.truckWeight_max;
            ez += std::max(0., overWeightTruck);
        }
    }
    for (int i = 0; i < droneTripList.size(); i++) {
        for (int j = 0; j < droneTripList[i].size(); j++) {
            if (droneTripList[i][j].empty()) {
                continue;
            }
            for (int k: droneTripList[i][j]) {
                cz += std::max(0.,
                               droneTripCompleteTime[i][j] - cusCompleteTime[k] - config.sampleLimitationWaitingTime);
                
            }
            droneWaitingTime[i][j] = droneTripCompleteTime[i][j] - cusCompleteTime[droneTripList[i][j][0]];
        }
    }
    for (int i = 0; i < droneTripList.size(); i++){
        for (int j = 0; j < droneTripList[i].size(); j++){
            if (droneTripList[i][j].empty()) {
                continue;
            }
            double overEnergydrone = (overEnergyComplete[i][j] - config.droneBatteryPower );
            ez += std::max(0., overEnergydrone);
            double overWeight = (totalDroneDemand[i][j] - config.droneCapacity);
            dz += std::max(0., overWeight);
        }
        
    }
    
    double point = c + alpha1 * cz + alpha2 * dz + ez ;
    return {{{point, c, cz, dz, ez}}, totalDroneDemand, totalTruckDemand, droneWaitingTime, truckWaitingTime, overEnergyComplete};
}

Solution::Solution(Config &config, Input &input, double alpha1, double alpha2) {
    this->input = input;
    this->config = config;
    for (int i = 0; i < config.numDrone; i++) {
        droneTripList.emplace_back();
    }

    for (int i = 0; i < config.numTech; i++) {
        techTripList.emplace_back();
    }

    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
}

Solution *Solution::relocate(const std::vector<std::string> &tabuList, double bestFeasibleScore, RouteType type, bool &isImproved) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore()[0][0][0];

    if (type != INTER) {
        // std::cout << "Inter start\n"; 
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++) {

                    if (xIndex != 0) {
                        Solution s = *this;

                        s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin(),
                                s.droneTripList[droneIndex][tripIndex][xIndex]);

                        s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);

                        double newScore = s.getScore()[0][0][0];
                        // if (newScore < curScore)
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                        //             s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                        //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = std::to_string(
                        //                 droneTripList[droneIndex][tripIndex][xIndex]);
                        //         curScore = newScore;
                        //     }
                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                            isImproved = true;
                            bestSolution->bestDroneTripList = s.droneTripList;
                            bestSolution->bestTechTripList = s.techTripList;
                            bestFeasibleScore = newScore;
                            if (curScore - newScore > config.tabuEpsilon){
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;
                            }
                            
                        }
                        
                        if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(
                                    s.droneTripList[droneIndex][tripIndex][xIndex]))){
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;    
                        }
                        
                    }

                    for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                        if (yIndex == xIndex || yIndex == xIndex - 1) {
                            continue;
                        }

                        Solution s = *this;

                        s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin() + yIndex + 1,
                                s.droneTripList[droneIndex][tripIndex][xIndex]);

                        if (xIndex < yIndex) {
                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                        } else {
                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                        }

                        double newScore = s.getScore()[0][0][0];

                        // if (newScore < curScore) {
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                        //             s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                        //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = std::to_string(
                        //                 droneTripList[droneIndex][tripIndex][xIndex]);
                        //         curScore = newScore;
                        //     }
                        // }
                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                            isImproved = true;
                            bestSolution->bestDroneTripList = s.droneTripList;
                            bestSolution->bestTechTripList = s.techTripList;
                            bestFeasibleScore = newScore;
                            if (curScore - newScore > config.tabuEpsilon){
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;
                            }
                            
                        }
                        
                        if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(
                                    s.droneTripList[droneIndex][tripIndex][xIndex]))){
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;    
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for(int tripIndex = 0; tripIndex < techTripList[techIndex].size(); tripIndex ++){
                for (int xIndex = 0; xIndex < techTripList[techIndex][tripIndex].size(); xIndex++) {
                    if (xIndex != 0) {
                        Solution s = *this;
                        s.techTripList[techIndex][tripIndex].insert(s.techTripList[techIndex][tripIndex].begin(),
                                                        s.techTripList[techIndex][tripIndex][xIndex]);


                        s.techTripList[techIndex][tripIndex].erase(s.techTripList[techIndex][tripIndex].begin() + xIndex + 1);

                        double newScore = s.getScore()[0][0][0];

                        // if (newScore < curScore) {
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                        //             s.techTripList[techIndex][tripIndex][xIndex])) &&
                        //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = std::to_string(techTripList[techIndex][tripIndex][xIndex]);
                        //         curScore = newScore;
                        //     }
                        // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][tripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.techTripList[techIndex][tripIndex][xIndex]))){
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                techTripList[techIndex][tripIndex][xIndex]);
                                    curScore = newScore;    
                                }                        
                    }

                    for (int yIndex = 0; yIndex < techTripList[techIndex][tripIndex].size(); yIndex++) {
                        if (yIndex == xIndex || yIndex == xIndex - 1) {
                            continue;
                        }

                        Solution s = *this;

                        s.techTripList[techIndex][tripIndex].insert(s.techTripList[techIndex][tripIndex].begin() + yIndex + 1,
                                                        s.techTripList[techIndex][tripIndex][xIndex]);

                        if (xIndex < yIndex) {
                            s.techTripList[techIndex][tripIndex].erase(
                                    s.techTripList[techIndex][tripIndex].begin() + xIndex);
                        } else {
                            s.techTripList[techIndex][tripIndex].erase(
                                    s.techTripList[techIndex][tripIndex].begin() + xIndex + 1);
                        }

                        double newScore = s.getScore()[0][0][0];

                        // if (newScore < curScore) { 
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                        //             s.techTripList[techIndex][tripIndex][xIndex])) &&
                        //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = std::to_string(techTripList[techIndex][tripIndex][xIndex]);
                        //         curScore = newScore;
                        //     }
                        // }
                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                            isImproved = true;
                            bestSolution->bestDroneTripList = s.droneTripList;
                            bestSolution->bestTechTripList = s.techTripList;
                            bestFeasibleScore = newScore;
                            if (curScore - newScore > config.tabuEpsilon){
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                        techTripList[techIndex][tripIndex][xIndex]);
                                curScore = newScore;
                            }
                            
                        }
                        
                        if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(
                                    s.techTripList[techIndex][tripIndex][xIndex]))){
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                        techTripList[techIndex][tripIndex][xIndex]);
                            curScore = newScore;    
                        }
                    }
                    // std::cout << "SS 1 : " << techIndex << tripIndex << xIndex << "\n";
                }
            }
            
        }
    }
    if (type != INTRA) {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++) {

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++) {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++) {
                            if (droneIndex2 == droneIndex && tripIndex == tripIndex2) {
                                continue;
                            }

                            Solution s = *this;
                            s.droneTripList[droneIndex2][tripIndex2].insert(
                                    s.droneTripList[droneIndex2][tripIndex2].begin(),
                                    s.droneTripList[droneIndex][tripIndex][xIndex]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                            double newScore = s.getScore()[0][0][0];

                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                            //             s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                            //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = std::to_string(
                            //                 droneTripList[droneIndex][tripIndex][xIndex]);

                            //         curScore = newScore;
                            //     }
                            // }
                            if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                isImproved = true;
                                bestSolution->bestDroneTripList = s.droneTripList;
                                bestSolution->bestTechTripList = s.techTripList;
                                bestFeasibleScore = newScore;
                                if (curScore - newScore > config.tabuEpsilon){
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                            droneTripList[droneIndex][tripIndex][xIndex]);
                                    curScore = newScore;
                                }
                                
                            }
                            
                            if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(
                                        s.droneTripList[droneIndex][tripIndex][xIndex]))){
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                            droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;    
                            }

                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                                s = *this;
                                s.droneTripList[droneIndex2][tripIndex2].insert(
                                        s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1,
                                        s.droneTripList[droneIndex][tripIndex][xIndex]);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                                newScore = s.getScore()[0][0][0];

                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                                //             s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = std::to_string(
                                //                 droneTripList[droneIndex][tripIndex][xIndex]);

                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.droneTripList[droneIndex][tripIndex][xIndex]))){
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                droneTripList[droneIndex][tripIndex][xIndex]);
                                    curScore = newScore;    
                                }
                            }
                        }
                        
                    }

                    //tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for(int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex ++){
                            Solution s = *this;
                            s.techTripList[techIndex][techTripIndex].insert(s.techTripList[techIndex][techTripIndex].begin(),
                                                            s.droneTripList[droneIndex][tripIndex][xIndex]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                            double newScore = s.getScore()[0][0][0];

                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.droneTripList[droneIndex][tripIndex][xIndex]))){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                droneTripList[droneIndex][tripIndex][xIndex]);
                                    curScore = newScore;    
                                }
                            for (int yIndex = 0; yIndex < techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                s = *this;

                                s.techTripList[techIndex][techTripIndex].insert(s.techTripList[techIndex][techTripIndex].begin() + yIndex + 1,
                                                                s.droneTripList[droneIndex][tripIndex][xIndex]);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                                newScore = s.getScore()[0][0][0];

                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                                //             s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = std::to_string(
                                //                 droneTripList[droneIndex][tripIndex][xIndex]);
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.droneTripList[droneIndex][tripIndex][xIndex]))){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                droneTripList[droneIndex][tripIndex][xIndex]);
                                    curScore = newScore;    
                                }                                
                            }
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex ++){
                for (int xIndex = 0; xIndex < techTripList[techIndex][techTripIndex].size(); xIndex++) {

                    // tech
                    for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                        for (int techTripIndex2 = 0; techTripIndex2 < techTripList[techIndex2].size(); techTripIndex2++){
                            if (techIndex == techIndex2 && techTripIndex == techTripIndex2) {
                                continue;
                            }

                            Solution s = *this;

                            s.techTripList[techIndex2][techTripIndex2].insert(s.techTripList[techIndex2][techTripIndex2].begin(),
                                                            s.techTripList[techIndex][techTripIndex][xIndex]);

                            s.techTripList[techIndex][techTripIndex].erase(
                                    s.techTripList[techIndex][techTripIndex].begin() + xIndex);


                            double newScore = s.getScore()[0][0][0];

                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                            //             s.techTripList[techIndex][techTripIndex][xIndex])) &&
                            //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                            //         curScore = newScore;
                            //     }
                            // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.techTripList[techIndex][techTripIndex][xIndex]))){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                techTripList[techIndex][techTripIndex][xIndex]);
                                    curScore = newScore;    
                                }

                            for (int yIndex = 0; yIndex < techTripList[techIndex2][techTripIndex2].size(); yIndex++) {
                                Solution s = *this;

                                s.techTripList[techIndex2][techTripIndex2].insert(s.techTripList[techIndex2][techTripIndex2].begin() + yIndex + 1,
                                                                s.techTripList[techIndex][techTripIndex][xIndex]);

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex);


                                double newScore = s.getScore()[0][0][0];

                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                                //             s.techTripList[techIndex][techTripIndex][xIndex])) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.techTripList[techIndex][techTripIndex][xIndex]))){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                techTripList[techIndex][techTripIndex][xIndex]);
                                    curScore = newScore;    
                                }

                            }
                        }
                    }

                    // drone
                    if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][xIndex]]) {
                        continue;
                    }

                    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                            Solution s = *this;

                            s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin(),
                                    s.techTripList[techIndex][techTripIndex][xIndex]);

                            s.techTripList[techIndex][techTripIndex].erase(
                                    s.techTripList[techIndex][techTripIndex].begin() + xIndex);


                            double newScore = s.getScore()[0][0][0];

                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                            //             s.techTripList[techIndex][techTripIndex][xIndex])) &&
                            //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                            //         curScore = newScore;
                            //     }
                            // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.techTripList[techIndex][techTripIndex][xIndex]))){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                techTripList[techIndex][techTripIndex][xIndex]);
                                    curScore = newScore;    
                                }                            

                            for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                                Solution s = *this;

                                s.droneTripList[droneIndex][tripIndex].insert(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex + 1,
                                        s.techTripList[techIndex][techTripIndex][xIndex]);

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex);


                                double newScore = s.getScore()[0][0][0];

                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, std::to_string(
                                //             s.techTripList[techIndex][techTripIndex][xIndex])) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    bestFeasibleScore = newScore;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, std::to_string(s.techTripList[techIndex][techTripIndex][xIndex]))){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                                techTripList[techIndex][techTripIndex][xIndex]);
                                    curScore = newScore;    
                                }                                
                            }
                        }
                        
                    }
                }
            }
        }

    }



    return bestSolution;
}

Solution *Solution::exchange(const std::vector<std::string> &tabuList, double bestFeasibleScore, RouteType type, bool &isImproved) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore()[0][0][0];

    if (type != INTRA) {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++) {

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++) {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++) {
                            if (droneIndex2 == droneIndex && tripIndex == tripIndex2) {
                                continue;
                            }

                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                                Solution s = *this;
                                std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                          s.droneTripList[droneIndex2][tripIndex2][yIndex]);
                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                std::string val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
            
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }                                
                            }
                        }
                    }

                    //tech
                    
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for(int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                            for (int yIndex = 0; yIndex < techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                

                                if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][yIndex]]) {
                                    continue;
                                }
                                Solution s = *this;
                                std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                        s.techTripList[techIndex][techTripIndex][yIndex]);

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][yIndex]);

                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                for (int xIndex = 0; xIndex < techTripList[techIndex][techTripIndex].size(); xIndex++) {

                    // tech
                    for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                        for (int techTripIndex2 = 0; techTripIndex2 < techTripList[techIndex2].size(); techTripIndex2++){    
                            if (techIndex == techIndex2 && techTripIndex == techTripIndex2) {
                                continue;
                            }
                        
                            for (int yIndex = 0; yIndex < techTripList[techIndex2][techTripIndex2].size(); yIndex++) {
                                Solution s = *this;

                                std::swap(s.techTripList[techIndex][techTripIndex][xIndex], s.techTripList[techIndex2][techTripIndex2][yIndex]);

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                std::string val2 = std::to_string(techTripList[techIndex2][techTripIndex2][yIndex]);

                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }

                    // drone
                    if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][xIndex]]) {
                        continue;
                    }

                    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                            for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                                Solution s = *this;

                                std::swap(s.techTripList[techIndex][techTripIndex][xIndex],
                                        s.droneTripList[droneIndex][tripIndex][yIndex]);

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);

                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (type != INTER) {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < (int) droneTripList[droneIndex][tripIndex].size() - 1; xIndex++) {
                    for (int yIndex = xIndex + 1; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                        Solution s = *this;

                        std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                  s.droneTripList[droneIndex][tripIndex][yIndex]);

                        double newScore = s.getScore()[0][0][0];
                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);

                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                for (int xIndex = 0; xIndex < (int) techTripList[techIndex][techTripIndex].size() - 1; xIndex++) {
                    for (int yIndex = xIndex + 1; yIndex < techTripList[techIndex][techTripIndex].size(); yIndex++) {
                        Solution s = *this;

                        std::swap(s.techTripList[techIndex][techTripIndex][xIndex],
                                s.techTripList[techIndex][techTripIndex][yIndex]);

                        double newScore = s.getScore()[0][0][0];
                        std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                        std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][yIndex]);

                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                    }
                }
            }
        }
    }


    return bestSolution;

}

//fix complete

bool Solution::checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val) {
    if (tabuList.empty()) {
        return true;
    } else {
        for (const std::string &s: tabuList) {
            if (s == val) {
                return false;
            }
        }
    }
    return true;
}

bool Solution::checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val1,
                                  const std::string &val2) {
    if (tabuList.empty()) {
        return true;
    } else {
        std::string tmp1 = val1 + "-" + val2;
        std::string tmp2 = val2 + "-" + val1;
        for (const std::string &s: tabuList) {
            if (s == tmp1 || s == tmp2) {
                return false;
            }
        }
    }
    return true;
}

bool
Solution::checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val1, const std::string &val2,
                             const std::string &val3) {
    if (tabuList.empty()) {
        return true;
    } else {
        std::string tmp = val1 + "-" + val2 + "-" + val3;
        for (const std::string &s: tabuList) {
            if (s == tmp) {
                return false;
            }
        }
    }
    return true;
}

Solution *Solution::orOpt(const std::vector<std::string> &tabuList, double bestFeasibleScore, bool &isImproved, RouteType type, int dis) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore()[0][0][0];

    if (type != INTRA) {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                if (droneTripList[droneIndex][tripIndex].size() <= dis) {
                    continue;
                }
                for (int xIndex = 0; xIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis; xIndex++) {
                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++) {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++) {
                            if (droneIndex2 == droneIndex && tripIndex == tripIndex2) {
                                continue;
                            }
                            Solution s = *this;
                            for (int i = dis; i >= 0; i--) {
                                s.droneTripList[droneIndex2][tripIndex2].insert(
                                        s.droneTripList[droneIndex2][tripIndex2].begin(),
                                        droneTripList[droneIndex][tripIndex][xIndex + i]);
                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex + i);
                            }

                            double newScore = s.getScore()[0][0][0];

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                            //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = "";
                            //         bestSolution->ext["state"] += val1;
                            //         bestSolution->ext["state"] += "-";
                            //         bestSolution->ext["state"] += val2;
                            //         curScore = newScore;
                            //     }
                            // }
                            if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                      
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                                s = *this;
                                for (int i = 0; i <= dis; i++) {
                                    s.droneTripList[droneIndex2][tripIndex2].insert(
                                            s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + i + 1,
                                            droneTripList[droneIndex][tripIndex][xIndex + i]);

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                            s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                                }

                                newScore = s.getScore()[0][0][0];

                                val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                              
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }

                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                            Solution s = *this;

                            for (int i = dis; i >= 0; i--) {
                                s.techTripList[techIndex][techTripIndex].insert(
                                        s.techTripList[techIndex][techTripIndex].begin(),
                                        droneTripList[droneIndex][tripIndex][xIndex + i]);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex + i);
                            }

                            double newScore = s.getScore()[0][0][0];

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                            //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = "";
                            //         bestSolution->ext["state"] += val1;
                            //         bestSolution->ext["state"] += "-";
                            //         bestSolution->ext["state"] += val2;
                            //         curScore = newScore;
                            //     }
                            // }
                            if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                 
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }

                            for (int yIndex = 0; yIndex < techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                s = *this;
                                for (int i = 0; i <= dis; i++) {
                                    s.techTripList[techIndex][techTripIndex].insert(
                                            s.techTripList[techIndex][techTripIndex].begin() + yIndex + i + 1,
                                            droneTripList[droneIndex][tripIndex][xIndex + i]);

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                            s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                                }

                                newScore = s.getScore()[0][0][0];

                                val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                               
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                if (techTripList[techIndex][techTripIndex].size() <= dis) {
                    continue;
                }
                for (int xIndex = 0; xIndex < (int) techTripList[techIndex][techTripIndex].size() - dis; xIndex++) {
                    //tech
                    for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                        for (int techTripIndex2 = 0; techTripIndex2 < techTripList[techIndex2].size(); techTripIndex2++){
                            if (techIndex == techIndex2 && techTripIndex == techTripIndex2) {
                                continue;
                            }

                            Solution s = *this;
                            for (int i = dis; i >= 0; i--) {
                                s.techTripList[techIndex2][techTripIndex2].insert(
                                        s.techTripList[techIndex2][techTripIndex2].begin(),
                                        techTripList[techIndex][techTripIndex][xIndex + i]);

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex + i);
                            }

                            double newScore = s.getScore()[0][0][0];

                            std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                            std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis]);
                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                            //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = "";
                            //         bestSolution->ext["state"] += val1;
                            //         bestSolution->ext["state"] += "-";
                            //         bestSolution->ext["state"] += val2;
                            //         curScore = newScore;
                            //     }
                            // }
                            if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                   
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }

                            for (int yIndex = 0; yIndex < techTripList[techIndex2][techTripIndex2].size(); yIndex++) {
                                s = *this;
                                for (int i = 0; i <= dis; i++) {
                                    s.techTripList[techIndex2][techTripIndex2].insert(
                                            s.techTripList[techIndex2][techTripIndex2].begin() + yIndex + i + 1,
                                            techTripList[techIndex][techTripIndex][xIndex + i]);

                                    s.techTripList[techIndex][techTripIndex].erase(
                                            s.techTripList[techIndex][techTripIndex].begin() + xIndex);
                                }

                                newScore = s.getScore()[0][0][0];

                                val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                val2 = std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                      
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }

                    //drone
                    bool canMoveToDroneTrip = true;
                    for (int i = xIndex; i < xIndex + dis; i++) {
                        if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][i]]) {
                            canMoveToDroneTrip = false;
                            break;
                        }
                    }
                    if (!canMoveToDroneTrip) {
                        continue;
                    }

                    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                            Solution s = *this;

                            for (int i = dis; i >= 0; i--) {
                                s.droneTripList[droneIndex][tripIndex].insert(
                                        s.droneTripList[droneIndex][tripIndex].begin(),
                                        techTripList[techIndex][techTripIndex][xIndex + i]);

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex + i);
                            }

                            double newScore = s.getScore()[0][0][0];

                            std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                            std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis]);
                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                            //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = "";
                            //         bestSolution->ext["state"] += val1;
                            //         bestSolution->ext["state"] += "-";
                            //         bestSolution->ext["state"] += val2;
                            //         curScore = newScore;
                            //     }
                            // }
                            if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                           
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                                s = *this;
                                for (int i = 0; i <= dis; i++) {
                                    s.droneTripList[droneIndex][tripIndex].insert(
                                            s.droneTripList[droneIndex][tripIndex].begin() + yIndex + i + 1,
                                            techTripList[techIndex][techTripIndex][xIndex + i]);

                                    s.techTripList[techIndex][techTripIndex].erase(
                                            s.techTripList[techIndex][techTripIndex].begin() + xIndex);
                                }

                                newScore = s.getScore()[0][0][0];

                                val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                val2 = std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                            
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (type != INTER) {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                if (droneTripList[droneIndex][tripIndex].size() <= dis) {
                    continue;
                }
                for (int xIndex = 0; xIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis; xIndex++) {
                    if (xIndex != 0) {
                        Solution s = *this;
                        for (int i = dis; i >= 0; i--) {
                            s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin(),
                                    droneTripList[droneIndex][tripIndex][xIndex + i]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis + 1);
                        }

                        double newScore = s.getScore()[0][0][0];

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        // if (newScore < curScore) {
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                        //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = "";
                        //         bestSolution->ext["state"] += val1;
                        //         bestSolution->ext["state"] += "-";
                        //         bestSolution->ext["state"] += val2;
                        //         curScore = newScore;
                        //     }
                        // }
                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                 
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                    }
                    for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                        if (yIndex == xIndex || (yIndex < xIndex && yIndex + dis >= xIndex)) {
                            continue;
                        }

                        Solution s = *this;

                        for (int i = 0; i <= dis; i++) {
                            s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin() + yIndex + i + 1,
                                    droneTripList[droneIndex][tripIndex][xIndex + i]);
                        }

                        for (int i = 0; i <= dis; i++) {
                            if (xIndex < yIndex) {
                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                            } else {
                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis + 1);
                            }
                        }

                        double newScore = s.getScore()[0][0][0];

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        // if (newScore < curScore) {
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                        //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = "";
                        //         bestSolution->ext["state"] += val1;
                        //         bestSolution->ext["state"] += "-";
                        //         bestSolution->ext["state"] += val2;
                        //         curScore = newScore;
                        //     }
                        // }
                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                 
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                if (techTripList[techIndex].size() <= dis) {
                    continue;
                }
                for (int xIndex = 0; xIndex < (int) techTripList[techIndex][techTripIndex].size() - dis; xIndex++) {
                    if (xIndex != 0) {
                        Solution s = *this;
                        for (int i = dis; i >= 0; i--) {
                            s.techTripList[techIndex][techTripIndex].insert(
                                    s.techTripList[techIndex][techTripIndex].begin(),
                                    techTripList[techIndex][techTripIndex][xIndex + i]);

                            s.techTripList[techIndex][techTripIndex].erase(
                                    s.techTripList[techIndex][techTripIndex].begin() + xIndex + dis + 1);
                        }

                        double newScore = s.getScore()[0][0][0];

                        std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                        std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis]);
                        // if (newScore < curScore) {
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                        //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = "";
                        //         bestSolution->ext["state"] += val1;
                        //         bestSolution->ext["state"] += "-";
                        //         bestSolution->ext["state"] += val2;
                        //         curScore = newScore;
                        //     }
                        // }
                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                  
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                    }

                    for (int yIndex = 0; yIndex < techTripList[techIndex][techTripIndex].size(); yIndex++) {
                        if (yIndex == xIndex || (yIndex < xIndex && yIndex + dis >= xIndex)) {
                            continue;
                        }

                        Solution s = *this;
                        for (int i = 0; i <= dis; i++) {
                            s.techTripList[techIndex][techTripIndex].insert(
                                    s.techTripList[techIndex][techTripIndex].begin() + yIndex + i + 1,
                                    techTripList[techIndex][techTripIndex][xIndex + i]);
                        }

                        for (int i = 0; i <= dis; i++) {
                            if (xIndex < yIndex) {
                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex);
                            } else {
                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex + dis + 1);
                            }
                        }
                        double newScore = s.getScore()[0][0][0];

                        std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                        std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis]);
                        // if (newScore < curScore) {
                        //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                        //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                        //         isImproved = true;
                        //         bestSolution->droneTripList = s.droneTripList;
                        //         bestSolution->techTripList = s.techTripList;
                        //         bestSolution->ext["state"] = "";
                        //         bestSolution->ext["state"] += val1;
                        //         bestSolution->ext["state"] += "-";
                        //         bestSolution->ext["state"] += val2;
                        //         curScore = newScore;
                        //     }
                        // }
                        if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                  
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                    }
                }
            }
        }
    }


    return bestSolution;

}



Solution *Solution::twoOpt(const std::vector<std::string> &tabuList, double bestFeasibleScore, bool &isImproved, RouteType type) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore()[0][0][0];

    if (type != INTRA) {
        // drone
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = -1; xIndex < (int) droneTripList[droneIndex][tripIndex].size(); xIndex++) {
                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                            for (int yIndex = -1; yIndex < (int) techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0) {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex][tripIndex].begin(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0) {
                                    tmp2.insert(tmp2.end(),
                                                techTripList[techIndex][techTripIndex].begin(),
                                                techTripList[techIndex][techTripIndex].begin() + yIndex + 1);
                                }

                                if (yIndex < (int) techTripList[techIndex][techTripIndex].size()) {
                                    tmp1.insert(tmp1.end(),
                                                techTripList[techIndex][techTripIndex].begin() + yIndex + 1,
                                                techTripList[techIndex][techTripIndex].end());
                                }

                                if (xIndex < (int) droneTripList[droneIndex][tripIndex].size()) {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1,
                                                droneTripList[droneIndex][tripIndex].end());
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex][tripIndex] = tmp1;
                                s.techTripList[techIndex][techTripIndex] = tmp2;

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(techTripList[techIndex][techTripIndex][yIndex]);
                                }
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                                }
                            }
                        }
                    }

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++) {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++) {
                            if (droneIndex == droneIndex2 && tripIndex == tripIndex2) {
                                continue;
                            }
                            for (int yIndex = -1;
                                 yIndex < (int) droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0) {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex][tripIndex].begin(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0) {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1);
                                }

                                if (yIndex < (int) droneTripList[droneIndex2][tripIndex2].size()) {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1,
                                                droneTripList[droneIndex2][tripIndex2].end());
                                }

                                if (xIndex < (int) droneTripList[droneIndex][tripIndex].size()) {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1,
                                                droneTripList[droneIndex][tripIndex].end());
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex][tripIndex] = tmp1;
                                s.droneTripList[droneIndex2][tripIndex2] = tmp2;

                                double newScore = s.getScore()[0][0][0];
                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                }
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                            if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                            }
                        }
                    }
                }
            }
        }

        // tech
        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                for (int xIndex = -1; xIndex < (int) techTripList[techIndex][techTripIndex].size(); xIndex++) {
                    // drone
                    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                            for (int yIndex = -1; yIndex < (int) droneTripList[droneIndex][tripIndex].size(); yIndex++) {

                                bool canMoveToDroneTrip = true;
                                for (int i = xIndex + 1; i < techTripList[techIndex][techTripIndex].size(); i++) {
                                    if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][i]]) {
                                        canMoveToDroneTrip = false;
                                        break;
                                    }
                                }
                                if (!canMoveToDroneTrip) {
                                    continue;
                                }

                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0) {
                                    tmp1.insert(tmp1.end(),
                                                techTripList[techIndex][techTripIndex].begin(),
                                                techTripList[techIndex][techTripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0) {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex][tripIndex].begin(),
                                                droneTripList[droneIndex][tripIndex].begin() + yIndex + 1);
                                }

                                if (yIndex < (int) droneTripList[droneIndex][tripIndex].size()) {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex][tripIndex].begin() + yIndex + 1,
                                                droneTripList[droneIndex][tripIndex].end());
                                }

                                if (xIndex < (int) techTripList[techIndex][techTripIndex].size()) {
                                    tmp2.insert(tmp2.end(),
                                                techTripList[techIndex][techTripIndex].begin() + xIndex + 1,
                                                techTripList[techIndex][techTripIndex].end());
                                }
                                Solution s = *this;

                                s.techTripList[techIndex][techTripIndex] = tmp1;
                                s.droneTripList[droneIndex][tripIndex] = tmp2;

                                double newScore = s.getScore()[0][0][0];
                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);
                                }
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                            if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                 
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                            }
                        }
                    }

                    // tech
                    for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                        for (int techTripIndex2 = 0; techTripIndex2 < techTripList[techIndex2].size(); techTripIndex2++){
                            if (techIndex == techIndex2 && techTripIndex2 == techTripIndex) {
                                continue;
                            }
                            for (int yIndex = -1; yIndex < (int) techTripList[techIndex2][techTripIndex2].size(); yIndex++) {
                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0) {
                                    tmp1.insert(tmp1.end(),
                                                techTripList[techIndex][techTripIndex].begin(),
                                                techTripList[techIndex][techTripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0) {
                                    tmp2.insert(tmp2.end(),
                                                techTripList[techIndex2][techTripIndex2].begin(),
                                                techTripList[techIndex2][techTripIndex2].begin() + yIndex + 1);
                                }

                                if (yIndex < (int) techTripList[techIndex2][techTripIndex2].size()) {
                                    tmp1.insert(tmp1.end(),
                                                techTripList[techIndex2][techTripIndex2].begin() + yIndex + 1,
                                                techTripList[techIndex2][techTripIndex2].end());
                                }

                                if (xIndex < (int) techTripList[techIndex][techTripIndex].size()) {
                                    tmp2.insert(tmp2.end(),
                                                techTripList[techIndex][techTripIndex].begin() + xIndex + 1,
                                                techTripList[techIndex][techTripIndex].end());
                                }
                                Solution s = *this;

                                s.techTripList[techIndex][techTripIndex] = tmp1;
                                s.techTripList[techIndex2][techTripIndex2] = tmp2;

                                double newScore = s.getScore()[0][0][0];
                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(techTripList[techIndex2][techTripIndex2][yIndex]);
                                }
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                            }
                        }
                    }
                }
            }
        }
    }

    if (type != INTER) {
        // drone
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = -1; xIndex < (int) droneTripList[droneIndex][tripIndex].size(); xIndex++) {
                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++) {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++) {
                            if (droneIndex == droneIndex2 && tripIndex == tripIndex2) {
                                continue;
                            }
                            for (int yIndex = -1;
                                 yIndex < (int) droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0) {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex][tripIndex].begin(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0) {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1);
                                }

                                if (yIndex < (int) droneTripList[droneIndex2][tripIndex2].size()) {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1,
                                                droneTripList[droneIndex2][tripIndex2].end());
                                }

                                if (xIndex < (int) droneTripList[droneIndex][tripIndex].size()) {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1,
                                                droneTripList[droneIndex][tripIndex].end());
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex][tripIndex] = tmp1;
                                s.droneTripList[droneIndex2][tripIndex2] = tmp2;

                                double newScore = s.getScore()[0][0][0];
                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                }
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                  
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                                }
                            }
                        }
                    }
                }
            }
        }

        // tech
        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                for (int xIndex = -1; xIndex < (int) techTripList[techIndex][techTripIndex].size(); xIndex++) {
                    
                    for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                        for (int techTripIndex2 = 0; techTripIndex2 < techTripList[techIndex2].size(); techTripIndex2++){
                            if (techIndex == techIndex2 && techTripIndex == techTripIndex2) {
                                continue;
                            }

                            for (int yIndex = -1; yIndex < (int) techTripList[techIndex2][techTripIndex2].size(); yIndex++) {
                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0) {
                                    tmp1.insert(tmp1.end(),
                                                techTripList[techIndex][techTripIndex].begin(),
                                                techTripList[techIndex][techTripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0) {
                                    tmp2.insert(tmp2.end(),
                                                techTripList[techIndex2][techTripIndex2].begin(),
                                                techTripList[techIndex2][techTripIndex2].begin() + yIndex + 1);
                                }

                                if (yIndex < (int) techTripList[techIndex2][techTripIndex2].size()) {
                                    tmp1.insert(tmp1.end(),
                                                techTripList[techIndex2][techTripIndex2].begin() + yIndex + 1,
                                                techTripList[techIndex2][techTripIndex2].end());
                                }

                                if (xIndex < (int) techTripList[techIndex][techTripIndex].size()) {
                                    tmp2.insert(tmp2.end(),
                                                techTripList[techIndex][techTripIndex].begin() + xIndex + 1,
                                                techTripList[techIndex][techTripIndex].end());
                                }

                                Solution s = *this;

                                s.techTripList[techIndex][techTripIndex] = tmp1;
                                s.techTripList[techIndex2][techTripIndex2] = tmp2;

                                double newScore = s.getScore()[0][0][0];
                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(techTripList[techIndex2][techTripIndex2][yIndex]);
                                }
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                                if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                           
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                                }
                            }
                        }

                    }
                }
            }
        }
    }



    return bestSolution;

}

Solution *Solution::crossExchange(const std::vector<std::string> &tabuList, double bestFeasibleScore, bool &isImproved, RouteType type, int dis1,
                                  int dis2) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore()[0][0][0];
    if (type != INTRA) {
        // drone

        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis1; xIndex++) {
                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                            for (int yIndex = 0; yIndex < (int) techTripList[techIndex][techTripIndex].size() - dis2; yIndex++) {
                                bool canMoveToDroneTrip = true;
                                for (int i = yIndex; i <= yIndex + dis2; i++) {
                                    if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][i]]) {
                                        canMoveToDroneTrip = false;
                                        break;
                                    }
                                }
                                if (!canMoveToDroneTrip) {
                                    continue;
                                }

                                Solution s = *this;

                                s.techTripList[techIndex][techTripIndex].insert(
                                        s.techTripList[techIndex][techTripIndex].begin() + yIndex + dis2 + 1,
                                        droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                        droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1
                                );
                                s.droneTripList[droneIndex][tripIndex].insert(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1,
                                        techTripList[techIndex][techTripIndex].begin() + yIndex,
                                        techTripList[techIndex][techTripIndex].begin() + yIndex + dis2 + 1
                                );

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + yIndex,
                                        s.techTripList[techIndex][techTripIndex].begin() + yIndex + dis2 + 1
                                );

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1
                                );

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex])
                                                + "-" +
                                                std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                                std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][yIndex])
                                                + "-" + std::to_string(techTripList[techIndex][techTripIndex][yIndex + dis2]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                            if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                            }
                        }
                    }

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++) {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++) {
                            if (droneIndex == droneIndex2 && tripIndex == tripIndex2) {
                                continue;
                            }
                            for (int yIndex = 0;
                                 yIndex < (int) droneTripList[droneIndex2][tripIndex2].size() - dis2; yIndex++) {
                                if (droneIndex == droneIndex2 && tripIndex == tripIndex2) {
                                    continue;
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex2][tripIndex2].insert(
                                        s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + dis2 + 1,
                                        droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                        droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1
                                );
                                s.droneTripList[droneIndex][tripIndex].insert(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1,
                                        droneTripList[droneIndex2][tripIndex2].begin() + yIndex,
                                        droneTripList[droneIndex2][tripIndex2].begin() + yIndex + dis2 + 1
                                );

                                s.droneTripList[droneIndex2][tripIndex2].erase(
                                        s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex,
                                        s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + dis2 + 1
                                );

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1
                                );

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex])
                                                   + "-" +
                                                   std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                                std::string val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex])
                                                   + "-" + std::to_string(
                                        droneTripList[droneIndex2][tripIndex2][yIndex + dis2]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                            if(s.checkFeasibleDroneTrip()){  
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                            }
                        }
                    }
                }
            }
        }

        // tech
        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for ( int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                for (int xIndex = 0; xIndex < (int) techTripList[techIndex][techTripIndex].size() - dis1; xIndex++) {

                    // drone
                    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                            for (int yIndex = 0;
                                yIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis2; yIndex++) {
                                bool canMoveToDroneTrip = true;
                                for (int i = xIndex; i <= xIndex + dis1; i++) {
                                    if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][i]]) {
                                        canMoveToDroneTrip = false;
                                        break;
                                    }
                                }
                                if (!canMoveToDroneTrip) {
                                    continue;
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex][tripIndex].insert(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1,
                                        techTripList[techIndex][techTripIndex].begin() + xIndex,
                                        techTripList[techIndex][techTripIndex].begin() + xIndex + dis1 + 1
                                );
                                s.techTripList[techIndex][techTripIndex].insert(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex + dis1 + 1,
                                        droneTripList[droneIndex][tripIndex].begin() + yIndex,
                                        droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1
                                );

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex,
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1
                                );

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex,
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex + dis1 + 1
                                );

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex])
                                                + "-" + std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis1]);
                                std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex])
                                                + "-" +
                                                std::to_string(droneTripList[droneIndex][tripIndex][yIndex + dis2]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                            if(s.checkFeasibleDroneTrip()){    
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                            }
                        }
                    }

                    // tech
                    for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                        for (int techTripIndex2 = 0; techTripIndex2 < techTripList[techIndex2].size(); techTripIndex2++){
                                for (int yIndex = 0; yIndex < (int) techTripList[techIndex2][techTripIndex2].size() - dis2; yIndex++) {
                                if (techIndex == techIndex2) {
                                    continue;
                                }
                                Solution s = *this;

                                s.techTripList[techIndex2][techTripIndex2].insert(
                                        s.techTripList[techIndex2][techTripIndex2].begin() + yIndex + dis2 + 1,
                                        techTripList[techIndex][techTripIndex].begin() + xIndex,
                                        techTripList[techIndex][techTripIndex].begin() + xIndex + dis1 + 1
                                );
                                s.techTripList[techIndex][techTripIndex].insert(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex + dis1 + 1,
                                        techTripList[techIndex2][techTripIndex2].begin() + yIndex,
                                        techTripList[techIndex2][techTripIndex2].begin() + yIndex + dis2 + 1
                                );

                                s.techTripList[techIndex2][techTripIndex2].erase(
                                        s.techTripList[techIndex2][techTripIndex2].begin() + yIndex,
                                        s.techTripList[techIndex2][techTripIndex2].begin() + yIndex + dis2 + 1
                                );

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex,
                                        s.techTripList[techIndex][techTripIndex].begin() + xIndex + dis1 + 1
                                );

                                double newScore = s.getScore()[0][0][0];

                                std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex])
                                                + "-" + std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis1]);
                                std::string val2 = std::to_string(techTripList[techIndex2][techTripIndex2][yIndex])
                                                + "-" +
                                                std::to_string(techTripList[techIndex2][techTripIndex2][yIndex + dis2]);
                                // if (newScore < curScore) {
                                //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                                //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                //         isImproved = true;
                                //         bestSolution->droneTripList = s.droneTripList;
                                //         bestSolution->techTripList = s.techTripList;
                                //         bestSolution->ext["state"] = "";
                                //         bestSolution->ext["state"] += val1;
                                //         bestSolution->ext["state"] += "-";
                                //         bestSolution->ext["state"] += val2;
                                //         curScore = newScore;
                                //     }
                                // }
                            if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                            }
                        }
                    }
                }
            }
        }
    }

    if (type != INTER) {
        // drone
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis1; xIndex++) {
                    for (int yIndex = 0; yIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis2; yIndex++) {
                        if (xIndex == yIndex) {
                            continue;
                        }
                        if (xIndex + dis1 < yIndex || xIndex > yIndex + dis2) {
                            Solution s = *this;
                            std::vector<int> tmp;

                            int beg1, end1, beg2, end2;
                            if (xIndex < yIndex) {
                                beg1 = xIndex;
                                end1 = xIndex + dis1 + 1;
                                beg2 = yIndex;
                                end2 = yIndex + dis2 + 1;
                            } else {
                                beg2 = xIndex;
                                end2 = xIndex + dis1 + 1;
                                beg1 = yIndex;
                                end1 = yIndex + dis2 + 1;
                            }

                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin(),
                                       droneTripList[droneIndex][tripIndex].begin() + beg1);
                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + beg2,
                                       droneTripList[droneIndex][tripIndex].begin() + end2);
                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + end1,
                                       droneTripList[droneIndex][tripIndex].begin() + beg2);
                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + beg1,
                                       droneTripList[droneIndex][tripIndex].begin() + end1);

                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + end2,
                                       droneTripList[droneIndex][tripIndex].end());

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin(),
                                    s.droneTripList[droneIndex][tripIndex].end());
                            s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin(),
                                    tmp.begin(), tmp.end());

                            double newScore = s.getScore()[0][0][0];

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex])
                                               + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex])
                                               + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][yIndex + dis2]);
                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                            //                                  abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = "";
                            //         bestSolution->ext["state"] += val1;
                            //         bestSolution->ext["state"] += "-";
                            //         bestSolution->ext["state"] += val2;
                            //         curScore = newScore;
                            //     }
                            // }
                        if(s.checkFeasibleDroneTrip()){
                            if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }  
                        }
                    }
                }
            }
        }

        // tech

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for ( int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                for (int xIndex = 0; xIndex < (int) techTripList[techIndex][techTripIndex].size() - dis1; xIndex++) {
                    for (int yIndex = 0; yIndex < (int) techTripList[techIndex][techTripIndex].size() - dis2; yIndex++) {
                        if (xIndex == yIndex) {
                            continue;
                        }
                        if (xIndex + dis1 < yIndex || xIndex > yIndex + dis2) {
                            Solution s = *this;

                            std::vector<int> tmp;

                            int beg1, end1, beg2, end2;
                            if (xIndex < yIndex) {
                                beg1 = xIndex;
                                end1 = xIndex + dis1 + 1;
                                beg2 = yIndex;
                                end2 = yIndex + dis2 + 1;
                            } else {
                                beg2 = xIndex;
                                end2 = xIndex + dis1 + 1;
                                beg1 = yIndex;
                                end1 = yIndex + dis2 + 1;
                            }

                            tmp.insert(tmp.end(),
                                    techTripList[techIndex][techTripIndex].begin(),
                                    techTripList[techIndex][techTripIndex].begin() + beg1);
                            tmp.insert(tmp.end(),
                                    techTripList[techIndex][techTripIndex].begin() + beg2,
                                    techTripList[techIndex][techTripIndex].begin() + end2);
                            tmp.insert(tmp.end(),
                                    techTripList[techIndex][techTripIndex].begin() + end1,
                                    techTripList[techIndex][techTripIndex].begin() + beg2);
                            tmp.insert(tmp.end(),
                                    techTripList[techIndex][techTripIndex].begin() + beg1,
                                    techTripList[techIndex][techTripIndex].begin() + end1);

                            tmp.insert(tmp.end(),
                                    techTripList[techIndex][techTripIndex].begin() + end2,
                                    techTripList[techIndex][techTripIndex].end());

                            s.techTripList[techIndex][techTripIndex].erase(s.techTripList[techIndex][techTripIndex].begin(),
                                                            s.techTripList[techIndex][techTripIndex].end());
                            s.techTripList[techIndex][techTripIndex].insert(s.techTripList[techIndex][techTripIndex].begin(),
                                                            tmp.begin(), tmp.end());

                            double newScore = s.getScore()[0][0][0];

                            std::string val1 = std::to_string(techTripList[techIndex][techTripIndex][xIndex])
                                            + "-" + std::to_string(techTripList[techIndex][techTripIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(techTripList[techIndex][techTripIndex][yIndex])
                                            + "-" +
                                            std::to_string(techTripList[techIndex][techTripIndex][yIndex + dis2]);
                            // if (newScore < curScore) {
                            //     if (newScore < bestFeasibleScore || (checkTabuCondition(tabuList, val1, val2) &&
                            //                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                            //         isImproved = true;
                            //         bestSolution->droneTripList = s.droneTripList;
                            //         bestSolution->techTripList = s.techTripList;
                            //         bestSolution->ext["state"] = "";
                            //         bestSolution->ext["state"] += val1;
                            //         bestSolution->ext["state"] += "-";
                            //         bestSolution->ext["state"] += val2;
                            //         curScore = newScore;
                            //     }
                            // }
                            if(s.checkFeasibleDroneTrip()){
                                if (s.checkFeasibleSolution() && bestFeasibleScore - newScore > config.tabuEpsilon) {
                                    isImproved = true;
                                    bestSolution->bestDroneTripList = s.droneTripList;
                                    bestSolution->bestTechTripList = s.techTripList;
                                    if (curScore - newScore > config.tabuEpsilon){
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                    
                                }
                                
                                if(curScore - newScore > config.tabuEpsilon && checkTabuCondition(tabuList, val1, val2)){
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if (isImproved) {
        return bestSolution;
    } else {
        return nullptr;
    }
}

std::string Solution::toString() {
    json jDrone(droneTripList);
    json jTech(techTripList);
    return jDrone.dump() + "::" + jTech.dump();
}

Solution Solution::ejection() {
    Solution s = *this;

    double bestGain = 0;
    std::vector<std::pair<std::vector<int>, std::vector<int>>> bestShiftSequence;
    double currentGain = 0;
    int currentLevel = 0;
    std::vector<std::pair<std::vector<int>, std::vector<int>>> shiftSequence;

    for (int droneIndex = 0; droneIndex < s.droneTripList.size(); droneIndex++) {
        for (int tripIndex = 0; tripIndex < s.droneTripList[droneIndex].size(); tripIndex++) {
            for (int xIndex = 0; xIndex < s.droneTripList[droneIndex][tripIndex].size(); xIndex++) {
                ejection(s,
                         {droneIndex, tripIndex, xIndex},
                         DRONE,
                         currentGain,
                         bestGain,
                         currentLevel,
                         shiftSequence,
                         bestShiftSequence);
            }
        }
    }

    for (int techIndex = 0; techIndex < s.techTripList.size(); techIndex++) {
        for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
            for (int xIndex = 0; xIndex < s.techTripList[techIndex][techTripIndex].size(); xIndex++) {
                ejection(s,
                        {techIndex, techTripIndex, xIndex},
                        TECHNICIAN,
                        currentGain,
                        bestGain,
                        currentLevel,
                        shiftSequence,
                        bestShiftSequence);
            }
        }
    }

    std::cout << "Now 2\n";
    for (std::pair<std::vector<int>, std::vector<int>> move: bestShiftSequence) {
        int x;
        if (move.first.size() == 2) {
            x = s.techTripList[move.first[0]][move.first[1]][move.first[2]];
            s.techTripList[move.first[0]][move.first[1]].erase(
                    s.techTripList[move.first[0]][move.first[1]].begin() + move.first[2]);
        } else {
            x = s.droneTripList[move.first[0]][move.first[1]][move.first[2]];
            s.droneTripList[move.first[0]][move.first[1]].erase(
                    s.droneTripList[move.first[0]][move.first[1]].begin() + move.first[2]);
        }

        if (move.second.size() == 2) {
            s.techTripList[move.second[0]][move.second[1]].insert(
                    s.techTripList[move.second[0]][move.second[1]].begin() + move.second[2], x);
        } else {
            s.droneTripList[move.second[0]][move.second[1]].insert(
                    s.droneTripList[move.second[0]][move.second[1]].begin() + move.second[2], x);
        }
    }
//    std::cout<< "-gain:" << bestGain<<std::endl;
       std::cout << "Now 3\n"; 
    return s;
}

std::vector<double> Solution::getScoreATrip(int tripIndex, TripType type) {
    std::vector<double> cusCompleteTime(input.numCus + 1, 0);
    double tmp, tmp1;
    double ct, dzt = 0, czt = 0;
    double allTechTime = 0, allDroneTime = 0;

    if (type == DRONE) {
        double droneCompleteTime = 0;
        int maxTrip = 0;
        for (auto &i: droneTripList) {
            if (maxTrip < i.size()) {
                maxTrip = (int) i.size();
            }
        }
        std::vector<double> droneTripCompleteTime(maxTrip, 0);
        tmp = 0;
        for (int j = 0; j < droneTripList[tripIndex].size(); j++) {
            if (droneTripList[tripIndex][j].empty()) {
                continue;
            }

            tmp1 = input.droneTimes[0][droneTripList[tripIndex][j][0]];
            cusCompleteTime[droneTripList[tripIndex][j][0]] = tmp1;

            for (int k = 0; k < (int) droneTripList[tripIndex][j].size() - 1; k++) {
                tmp1 += input.droneTimes[droneTripList[tripIndex][j][k]][droneTripList[tripIndex][j][k + 1]];
                cusCompleteTime[droneTripList[tripIndex][j][k + 1]] = tmp1;
            }
            droneTripCompleteTime[j] = tmp1 + input.droneTimes[droneTripList[tripIndex][j].back()][0];
            tmp += droneTripCompleteTime[j];
        }
        droneCompleteTime = tmp;
        if (tmp > allDroneTime) {
            allDroneTime = tmp;
        }

        for (int j = 0; j < droneTripList[tripIndex].size(); j++) {
            if (droneTripList[tripIndex][j].empty()) {
                continue;
            }
            for (int k: droneTripList[tripIndex][j]) {
                czt += std::max(0.,
                                droneTripCompleteTime[j] - cusCompleteTime[k] - config.sampleLimitationWaitingTime);
            }
            dzt += std::max(0., droneTripCompleteTime[j] - config.droneLimitationFightTime);
        }
    } else {
        double techCompleteTime = 0;
        int maxTechTrip = 0;
        for (auto &i : techTripList){
            if ( maxTechTrip < i.size()){
                maxTechTrip = (int) i.size();
            }
        }
        std::vector<double> techTripCompleteTime(config.numTech, 0);
        tmp = 0;
        for ( int j = 0; j < techTripList[tripIndex].size(); j ++){
            if (techTripList[tripIndex][j].empty()) {
                continue;
            }
            tmp1 = countTimeTruck(0, input.distances[0][techTripList[tripIndex][j][0]], input) 
                + input.serviceTimeByTruck[techTripList[tripIndex][j][0]]; 
            cusCompleteTime[techTripList[tripIndex][j][0]] = tmp1;

            for (int k = 0; k < (int) techTripList[tripIndex][j].size() - 1; k++) {
                tmp1 += countTimeTruck(tmp1, input.distances[techTripList[tripIndex][j][k]][techTripList[tripIndex][j][k+1]], input)
                    + input.serviceTimeByTruck[techTripList[tripIndex][j][k+1]]; 
                cusCompleteTime[techTripList[tripIndex][j][k + 1]] = tmp1;
            }
            techTripCompleteTime[j] = tmp1 + countTimeTruck(tmp1, input.distances[techTripList[tripIndex][j].back()][0], input); 
            tmp += techTripCompleteTime[j];
        }
        techCompleteTime = tmp;
        if (tmp > allTechTime) {
            allTechTime = tmp;
        }
        for (int j = 0; j < techTripList[tripIndex].size(); j++) {
            if (techTripList[tripIndex][j].empty()) {
                continue;
            }
            for (int k: techTripList[tripIndex][j]) {
                czt += std::max(0.,
                                techTripCompleteTime[j] - cusCompleteTime[k] - config.sampleLimitationWaitingTime);
            }
            dzt += std::max(0., techTripCompleteTime[j] - config.droneLimitationFightTime);
        }
    }
    ct = std::max(allDroneTime, allTechTime);
    return {ct, dzt, czt};
}

void
Solution::ejection(Solution &solution, std::vector<int> xIndex, TripType type, double gain, double &bestGain,
                   int &level,
                   std::vector<std::pair<std::vector<int>, std::vector<int>>> &shiftSequence,
                   std::vector<std::pair<std::vector<int>, std::vector<int>>> &bestShiftSequence) {
                
    std::vector<double> droneScores;
    double maxDroneScore = 0;
    std::cout << "Now 1 \n";
    droneScores.reserve(solution.droneTripList.size());
    int maxDroneIndex;
    for (int droneIndex = 0; droneIndex < solution.droneTripList.size(); droneIndex++) {
        double sc = solution.getScoreATrip(droneIndex, DRONE)[0];

        droneScores.push_back(sc);
        if (sc > maxDroneScore) {
            maxDroneScore = sc;
            maxDroneIndex = droneIndex;
        }
    }

    if (type == DRONE && xIndex[0] != maxDroneIndex) {
        return;
    }

    std::vector<double> techScores;
    double maxTechScore = 0;
    techScores.reserve(solution.techTripList.size());
    int maxTechIndex;
    for (int techIndex = 0; techIndex < solution.techTripList.size(); techIndex++) {
        double sc = solution.getScoreATrip(techIndex, TECHNICIAN)[0];
        techScores.push_back(sc);
        if (sc > maxTechScore) {
            maxTechScore = sc;
            maxTechIndex = techIndex;
        }
    }

    if (type == TECHNICIAN && xIndex[0] != maxTechIndex) {
        return;
    }

    if (maxDroneScore > maxTechScore && type == TECHNICIAN) {
        return;
    } else if (maxTechScore > maxDroneScore && type == DRONE) {
        return;
    }
    double fScore = std::max(maxDroneScore, maxTechScore);

    int x, predecessor, successor;
    std::vector<std::vector<double>> times;

    if (type == DRONE) {
        times = input.droneTimes;
        x = solution.droneTripList[xIndex[0]][xIndex[1]][xIndex[2]];
        predecessor = (xIndex[2] == 0) ? 0 : solution.droneTripList[xIndex[0]][xIndex[1]][xIndex[2] - 1];
        successor = (xIndex[2] == solution.droneTripList[xIndex[0]][xIndex[1]].size() - 1)
                    ? input.numCus + 1
                    : solution.droneTripList[xIndex[0]][xIndex[1]][xIndex[2] + 1];
        droneScores[xIndex[0]] -= times[predecessor][x] + times[x][successor] - times[predecessor][successor];

        double cScore = std::max(maxTechScore, *std::max_element(droneScores.begin(), droneScores.end()));
        double g = fScore - cScore;
        solution.droneTripList[xIndex[0]][xIndex[1]].erase(
                solution.droneTripList[xIndex[0]][xIndex[1]].begin() + xIndex[2]);
        gain += g;

        for (int droneIndex = 0; droneIndex < solution.droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < solution.droneTripList[droneIndex].size(); tripIndex++) {
                if (droneIndex == xIndex[0] && tripIndex == xIndex[1]) {
                    continue;
                }

                for (int cusIndex = 0; cusIndex < solution.droneTripList[droneIndex][tripIndex].size(); cusIndex++) {
                    int cus = solution.droneTripList[droneIndex][tripIndex][cusIndex];
                    int cusPredecessor = (cusIndex == 0) ? 0 : solution.droneTripList[droneIndex][tripIndex][cusIndex -
                                                                                                             1];
                    double delta = input.droneTimes[cusPredecessor][x]
                                   + input.droneTimes[x][cus] - input.droneTimes[cusPredecessor][cus];
                    double d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                    if (gain - d > bestGain) {
                        solution.droneTripList[droneIndex][tripIndex].insert(
                                solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                        if (tmp[1] == 0 && tmp[2] == 0) {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        } else if (level + 1 <= config.maxEjectionLevel) {
                            for (int yIndex = 0;
                                 yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                                Solution s = solution;

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(droneIndex, DRONE);
                                if (tmp[1] == 0 && tmp[2] == 0) {
                                    ejection(solution, {droneIndex, tripIndex, yIndex},
                                             DRONE,
                                             gain,
                                             bestGain,
                                             level,
                                             shiftSequence,
                                             bestShiftSequence);
                                }
                            }
                        }

                        solution.droneTripList[droneIndex][tripIndex].erase(
                                solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex);
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }


                    if (cusIndex == solution.droneTripList[droneIndex][tripIndex].size() - 1) {
                        delta = input.droneTimes[cus][x]
                                + input.droneTimes[x][input.numCus + 1] - input.droneTimes[cus][input.numCus + 1];
                        d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                        if (gain - d > bestGain) {
                            solution.droneTripList[droneIndex][tripIndex].insert(
                                    solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex + 1, x);
                            gain -= d;

                            shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex + 1}});
                            level++;
                            std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                            if (tmp[1] == 0 && tmp[2] == 0) {
                                bestShiftSequence = shiftSequence;
                                bestGain = gain;
                            } else if (level + 1 <= config.maxEjectionLevel) {
                                for (int yIndex = 0;
                                     yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                                    Solution s = solution;

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                            s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                    tmp = s.getScoreATrip(droneIndex, DRONE);
                                    if (tmp[1] == 0 && tmp[2] == 0) {
                                        ejection(solution, {droneIndex, tripIndex, yIndex},
                                                 DRONE,
                                                 gain,
                                                 bestGain,
                                                 level,
                                                 shiftSequence,
                                                 bestShiftSequence);
                                    }
                                }
                            }

                            solution.droneTripList[droneIndex][tripIndex].erase(
                                    solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex);
                            gain += d;
                            shiftSequence.pop_back();
                            level--;
                        }
                    }

                }
            }
        }

        for (int techIndex = 0; techIndex < solution.techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                for (int cusIndex = 0; cusIndex < solution.techTripList[techIndex][techTripIndex].size(); cusIndex++) {
                    int cus = solution.techTripList[techIndex][techTripIndex][cusIndex];
                    int cusPredecessor = (cusIndex == 0) ? 0 : solution.techTripList[techIndex][techTripIndex][cusIndex - 1];
                    double delta = input.techTimes[cusPredecessor][x]
                                + input.techTimes[x][cus] - input.techTimes[cusPredecessor][cus];
                    double d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                    if (gain - d > bestGain) {
                        solution.techTripList[techIndex][techTripIndex].insert(
                                solution.techTripList[techIndex][techTripIndex].begin() + cusIndex, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {techIndex, techTripIndex, cusIndex}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                        if (tmp[1] == 0 && tmp[2] == 0) {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        } else if (level + 1 <= config.maxEjectionLevel) {
                            for (int yIndex = 0;
                                yIndex < solution.techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                Solution s = solution;

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                                if (tmp[1] == 0 && tmp[2] == 0) {
                                    ejection(solution, {techIndex, yIndex},
                                            TECHNICIAN,
                                            gain,
                                            bestGain,
                                            level,
                                            shiftSequence,
                                            bestShiftSequence);
                                }
                            }
                        }

                        solution.techTripList[techIndex][techTripIndex].erase(
                                solution.techTripList[techIndex][techTripIndex].begin() + cusIndex);
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }

                    if (cusIndex == solution.techTripList[techIndex][techTripIndex].size() - 1) {
                        delta = input.techTimes[cus][x]
                                + input.techTimes[x][input.numCus + 1] - input.techTimes[cus][input.numCus + 1];
                        d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                        if (gain - d > bestGain) {
                            solution.techTripList[techIndex][techTripIndex].insert(
                                    solution.techTripList[techIndex][techTripIndex].begin() + cusIndex + 1, x);
                            gain -= d;

                            shiftSequence.push_back({xIndex, {techIndex, cusIndex + 1}});
                            level++;
                            std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                            if (tmp[1] == 0 && tmp[2] == 0) {
                                bestShiftSequence = shiftSequence;
                                bestGain = gain;
                            } else if (level + 1 <= config.maxEjectionLevel) {
                                for (int yIndex = 0;
                                    yIndex < solution.techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                    Solution s = solution;

                                    s.techTripList[techIndex][techTripIndex].erase(
                                            s.techTripList[techIndex][techTripIndex].begin() + yIndex);
                                    tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                                    if (tmp[1] == 0 && tmp[2] == 0) {
                                        ejection(solution, {techIndex, yIndex},
                                                TECHNICIAN,
                                                gain,
                                                bestGain,
                                                level,
                                                shiftSequence,
                                                bestShiftSequence);
                                    }
                                }
                            }

                            solution.techTripList[techIndex][techTripIndex].erase(
                                    solution.techTripList[techIndex][techTripIndex].begin() + cusIndex);
                            gain += d;
                            shiftSequence.pop_back();
                            level--;
                        }
                    }
                }
            }
        }

        solution.droneTripList[xIndex[0]][xIndex[1]].insert(
                solution.droneTripList[xIndex[0]][xIndex[1]].begin() + xIndex[2], x);
    } else {
        times = input.techTimes;
        x = solution.techTripList[xIndex[0]][xIndex[1]][xIndex[2]];
        predecessor = (xIndex[2] == 0) ? 0 : solution.techTripList[xIndex[0]][xIndex[1]][xIndex[2] - 1];
        
        successor = (xIndex[2] == solution.techTripList[xIndex[0]][xIndex[1]].size() - 1)
                    ? input.numCus + 1
                    : solution.techTripList[xIndex[0]][xIndex[1]][xIndex[2] + 1];
        techScores[xIndex[0]] -= times[predecessor][x] + times[x][successor] - times[predecessor][successor];

        double cScore = std::max(maxDroneScore, *std::max_element(techScores.begin(), techScores.end()));
        double g = fScore - cScore;
        solution.techTripList[xIndex[0]][xIndex[1]].erase(
                solution.techTripList[xIndex[0]][xIndex[1]].begin() + xIndex[2]);
        gain += g;
        for (int droneIndex = 0; droneIndex < solution.droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < solution.droneTripList[droneIndex].size(); tripIndex++) {
                for (int cusIndex = 0; cusIndex < solution.droneTripList[droneIndex][tripIndex].size(); cusIndex++) {
                    int cus = solution.droneTripList[droneIndex][tripIndex][cusIndex];
                    int cusPredecessor = (cusIndex == 0) ? 0 : solution.droneTripList[droneIndex][tripIndex][cusIndex -
                                                                                                             1];
                    double delta = input.droneTimes[cusPredecessor][x]
                                   + input.droneTimes[x][cus] - input.droneTimes[cusPredecessor][cus];
                    double d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                    if (gain - d > bestGain) {
                        solution.droneTripList[droneIndex][tripIndex].insert(
                                solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                        if (tmp[1] == 0 && tmp[2] == 0) {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        } else if (level + 1 <= config.maxEjectionLevel) {
                            for (int yIndex = 0;
                                 yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                                Solution s = solution;

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(droneIndex, DRONE);
                                if (tmp[1] == 0 && tmp[2] == 0) {
                                    ejection(solution, {droneIndex, tripIndex, yIndex},
                                             DRONE,
                                             gain,
                                             bestGain,
                                             level,
                                             shiftSequence,
                                             bestShiftSequence);
                                }
                            }
                        }

                        solution.droneTripList[droneIndex][tripIndex].erase(
                                solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex);
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }

                    if (cusIndex == solution.droneTripList[droneIndex][tripIndex].size() - 1) {
                        delta = input.droneTimes[cus][x]
                                + input.droneTimes[x][input.numCus + 1] - input.droneTimes[cus][input.numCus + 1];
                        d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                        if (gain - d > bestGain) {
                            solution.droneTripList[droneIndex][tripIndex].insert(
                                    solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex + 1, x);
                            gain -= d;

                            shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex + 1}});
                            level++;
                            std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                            if (tmp[1] == 0 && tmp[2] == 0) {
                                bestShiftSequence = shiftSequence;
                                bestGain = gain;
                            } else if (level + 1 <= config.maxEjectionLevel) {
                                for (int yIndex = 0;
                                     yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                                    Solution s = solution;

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                            s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                    tmp = s.getScoreATrip(droneIndex, DRONE);
                                    if (tmp[1] == 0 && tmp[2] == 0) {
                                        ejection(solution, {droneIndex, tripIndex, yIndex},
                                                 DRONE,
                                                 gain,
                                                 bestGain,
                                                 level,
                                                 shiftSequence,
                                                 bestShiftSequence);
                                    }
                                }
                            }

                            solution.droneTripList[droneIndex][tripIndex].erase(
                                    solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex);
                            gain += d;
                            shiftSequence.pop_back();
                            level--;
                        }
                    }
                }
            }
        }
        for (int techIndex = 0; techIndex < solution.techTripList.size(); techIndex++) {
            for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
                if (techIndex == xIndex[0]) {
                    continue;
                }
                for (int cusIndex = 0; cusIndex < solution.techTripList[techIndex][techTripIndex].size(); cusIndex++) {
                    int cus = solution.techTripList[techIndex][techTripIndex][cusIndex];
                    int cusPredecessor = (cusIndex == 0) ? 0 : solution.techTripList[techIndex][techTripIndex][cusIndex - 1];
                    double delta = input.techTimes[cusPredecessor][x]
                                + input.techTimes[x][cus] - input.techTimes[cusPredecessor][cus];
                    double d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                    if (gain - d > bestGain) {
                        solution.techTripList[techIndex][techTripIndex].insert(
                                solution.techTripList[techIndex][techTripIndex].begin() + cusIndex, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {techIndex, techTripIndex, cusIndex}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                        if (tmp[1] == 0 && tmp[2] == 0) {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        } else if (level + 1 <= config.maxEjectionLevel) {
                            for (int yIndex = 0;
                                yIndex < solution.techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                Solution s = solution;

                                s.techTripList[techIndex][techTripIndex].erase(
                                        s.techTripList[techIndex][techTripIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                                if (tmp[1] == 0 && tmp[2] == 0) {
                                    ejection(solution, {techIndex, yIndex},
                                            TECHNICIAN,
                                            gain,
                                            bestGain,
                                            level,
                                            shiftSequence,
                                            bestShiftSequence);
                                }
                            }
                        }

                        solution.techTripList[techIndex][techTripIndex].erase(
                                solution.techTripList[techIndex][techTripIndex].begin() + cusIndex);
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }

                    if (cusIndex == solution.techTripList[techIndex][techTripIndex].size() - 1) {
                        delta = input.techTimes[cus][x]
                                + input.techTimes[x][input.numCus + 1] - input.techTimes[cus][input.numCus + 1];
                        d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                        if (gain - d > bestGain) {
                            solution.techTripList[techIndex][techTripIndex].insert(
                                    solution.techTripList[techIndex][techTripIndex].begin() + cusIndex + 1, x);
                            gain -= d;

                            shiftSequence.push_back({xIndex, {techIndex, techTripIndex, cusIndex + 1}});
                            level++;
                            std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                            if (tmp[1] == 0 && tmp[2] == 0) {
                                bestShiftSequence = shiftSequence;
                                bestGain = gain;
                            } else if (level + 1 <= config.maxEjectionLevel) {
                                for (int yIndex = 0;
                                    yIndex < solution.techTripList[techIndex][techTripIndex].size(); yIndex++) {
                                    Solution s = solution;

                                    s.techTripList[techIndex][techTripIndex].erase(
                                            s.techTripList[techIndex][techTripIndex].begin() + yIndex);
                                    tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                                    if (tmp[1] == 0 && tmp[2] == 0) {
                                        ejection(solution, {techIndex, yIndex},
                                                TECHNICIAN,
                                                gain,
                                                bestGain,
                                                level,
                                                shiftSequence,
                                                bestShiftSequence);
                                    }
                                }
                            }

                            solution.techTripList[techIndex][techTripIndex].erase(
                                    solution.techTripList[techIndex][techTripIndex].begin() + cusIndex);
                            gain += d;
                            shiftSequence.pop_back();
                            level--;
                        }
                    }
                }
            }
        }

        solution.techTripList[xIndex[0]][xIndex[1]].insert(
                solution.techTripList[xIndex[0]][xIndex[1]].begin() + xIndex[2], x);
    }
}

void Solution::perturbation() {
    double swapRate = 0.01;
    int num = 0;
    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
            for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++) {

                // drone
                for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++) {
                    for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++) {
                        if (droneIndex2 == droneIndex && tripIndex == tripIndex2) {
                            continue;
                        }

                        for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                            if (Random::get(0.0, 1.0) > swapRate) {
                                continue;
                            }
                            std::swap(droneTripList[droneIndex][tripIndex][xIndex],
                                      droneTripList[droneIndex2][tripIndex2][yIndex]);
                            num++;
                        }
                    }
                }

                //tech
                for (auto &techIndex: techTripList) {
                    for ( auto &techTripIndex : techIndex){
                        for (int &yIndex: techTripIndex) {
                            if (Random::get(0.0, 1.0) > swapRate) {
                                continue;
                            }
                            std::swap(droneTripList[droneIndex][tripIndex][xIndex],
                                    yIndex);
                            num++;
                        }
                    }
                }
            }
        }
    }

    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
        for (int techTripIndex = 0; techTripIndex < techTripList[techIndex].size(); techTripIndex++){
            for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++) {

                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                    for (int techTripIndex2 = 0; techTripIndex2 < techTripList[techIndex2].size(); techTripIndex2++){
                        if (techIndex == techIndex2 || techTripIndex2 == techTripIndex) {
                            continue;
                        }
                        for (int yIndex = 0; yIndex < techTripList[techIndex2][techTripIndex2].size(); yIndex++) {
                            if (Random::get(0.0, 1.0) > swapRate) {
                                continue;
                            }

                            std::swap(techTripList[techIndex][techTripIndex][xIndex], techTripList[techIndex2][techTripIndex2][yIndex]);
                            num++;
                        }
                    }
                }

                // drone
                if (input.cusOnlyServedByTech[techTripList[techIndex][techTripIndex][xIndex]]) {
                    continue;
                }

                for (auto &droneIndex: droneTripList) {
                    for (auto &tripIndex: droneIndex) {
                        for (int &yIndex: tripIndex) {
                            if (Random::get(0.0, 1.0) > swapRate) {
                                continue;
                            }

                            std::swap(techTripList[techIndex][techTripIndex][xIndex],
                                    yIndex);
                            num++;
                        }
                    }
                }
            }
        }
    }

    for (auto &droneIndex: droneTripList) {
        for (auto &tripIndex: droneIndex) {
            for (int xIndex = 0; xIndex < (int) tripIndex.size() - 1; xIndex++) {
                for (int yIndex = xIndex + 1; yIndex < tripIndex.size(); yIndex++) {
                    if (Random::get(0.0, 1.0) > swapRate) {
                        continue;
                    }

                    std::swap(tripIndex[xIndex],
                              tripIndex[yIndex]);
                    num++;
                }
            }
        }
    }

    for (auto &techIndex: techTripList) {
        for (int xIndex = 0; xIndex < (int) techIndex.size() - 1; xIndex++) {
            for (int yIndex = xIndex + 1; yIndex < techIndex.size(); yIndex++) {
                if (Random::get(0.0, 1.0) > swapRate) {
                    continue;
                }

                std::swap(techIndex[xIndex],
                          techIndex[yIndex]);
                num++;
            }
        }
    }
    std::cout << "swap: " << num << std::endl;
}
void Solution::logConsole(){
    int i = 1;
    int j ;
    std::cout << "Alpha = " << alpha << "\n";
    std::cout << "G = " << g << "\n";
    std::cout << " \nDrone Trip Init : " << droneTripList.size() << std::endl;
    for(auto trips : droneTripList){
        std::cout << "Drone " << i << ": \n";
        j = 1;
        for(auto trip : trips){
            std::cout << "Trip " << j << ":";
            for (auto x : trip){
                std::cout <<" | " << x ;
            }
            j++;
            std::vector<int> drone_trip(trip);
            drone_trip.insert(drone_trip.begin(), 0);
            drone_trip.push_back(0);
            double energy = totalTripEnergy(drone_trip, input);
            std::cout << std::setprecision(10);
            std::cout << " \nTrip energy : " << energy << "\n";
        }
        i++;
    }
    i = 1;
    std::cout << " \nTruck Trip Init : " << techTripList.size() << std::endl;
    for(auto techTrips : techTripList){
        std::cout << "Truck " << i << "\n";
        j = 1;
        for(auto techTrip : techTrips){
            std::cout << "Trip " << j << ":";
            for(auto trip : techTrip){
                std:: cout << " | " << trip;
            }
            j++;
            std::cout <<"\n";
        }
    }
    std::cout << " \n "  << std::endl;
}

bool Solution::checkFeasibleDroneTrip(){
    Solution solution = *this;
    std::vector<std::vector<std::vector<int>>> droneTripList = solution.droneTripList;
    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
            for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++) {
                if (input.cusOnlyServedByTech[droneTripList[droneIndex][tripIndex][xIndex]])
                {
                    return false;
                }
                
            }
        }
    }
    return true;
}
bool Solution::checkFeasibleSolution(){
    Solution solution = *this;
    std::vector<std::vector<std::vector<double>>> solutionScore = solution.getScore();
    if (solutionScore[0][0][2] == 0 && solutionScore[0][0][3] == 0 && solutionScore[0][0][4] == 0){
        return true;
    }
    return false;
    
}

Solution::Solution() = default;




