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

double k1 = 0.8554;
double k2 = 0.3051;
double c1 = 2.8037;
double c2 = 0.3177;
double c4 = 0.0296;
double c5 = 0.0279;

double W = 1.5; // drone weight; 
double g = 9.8; // gravitational constant;
double alpha = 0.1745329252; // angle off attack;
double landing_speed = 5;
double takeoff_speed = 10;
double drone_speed = 16.66666666666; //(m/s)
double h = 200;
double drone_energy = 5000000;

double landing_Takeoff(double demand, double speed){
    double P;
    P = k1*(W + demand)*g*(speed/2 + sqrt(pow((speed/2), 2.) + (W + demand)*g/pow(k2, 2.))) + c2*sqrt(pow(((W + demand)*g), 3.));
    // printf("LANDING %0.15f \n", P);
    return P;
}

double horizontal(double demand, double speed){
    double P;
    double P1 = pow(((W + demand)*g - c5*pow((speed*cos(alpha)), 2.)), 2.);
    double P2 = pow(c4*pow(speed, 2.), 2.);
    double P3 = c4*pow(speed, 3);
    double P4 = pow(speed, 3.);
    P = (c1 + c2)*pow((P1 + P2 ), 3/4.) + P3;
    // printf(" During : %0.15f\n", P);
    return P;
}

double check_Capacity(std::vector<int> trip , Input input){
    double total_demand = 0;
    double time = 0;
    double total_Energy = landing_Takeoff(0, takeoff_speed)*h/takeoff_speed;
    for (int i = 0; i < trip.size() - 1; i++){
        time = input.distances[trip[i]][trip[i + 1]] * 1000 / drone_speed;
        total_demand += input.demand[trip[i]];
        total_Energy += horizontal(total_demand, drone_speed) * time ;
        
    }
    
    return total_Energy;
}

Solution *Solution::initSolution(Config &config, Input &input, InitType type, double alpha1, double alpha2) {
    Solution *best = nullptr, *current;
    double bestScore = std::numeric_limits<double>::max(), currentScore;
    if (type != DISTANCE) {
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(true, 1);
        currentScore = current->getScore();
        if (currentScore < bestScore) {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(false, 1);
        currentScore = current->getScore();
        if (currentScore < bestScore) {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(false, -1);
        currentScore = current->getScore();
        if (currentScore < bestScore) {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(true, -1);
        currentScore = current->getScore();
        if (currentScore < bestScore) {
            best = current;
            bestScore = currentScore;
        }
    }
    if (type != ANGLE) {
        current = new Solution(config, input, alpha1, alpha2);
        current->initByDistance(false);
        currentScore = current->getScore();
        if (currentScore < bestScore) {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByDistance(true);
        currentScore = current->getScore();
        if (currentScore < bestScore) {
            best = current;
            bestScore = currentScore;
        }
    }
    best->logConsole();
    return best;
}

void Solution::initByDistance(bool reverse) {
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

            double timeGoToFirstCus = time[0][nextCus];
            if (!droneTripList[index].back().empty()) {
                timeGoToFirstCus = time[0][droneTripList[index].back()[0]];
            }

            double flightTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0];
            double waitTime = flightTime - timeGoToFirstCus;
            std::vector<int> test_trip(droneTripList[i].back());
            test_trip.push_back(nextCus);
            double energy = check_Capacity(test_trip, input);

            if (flightTime > config.droneLimitationFightTime || waitTime > config.sampleLimitationWaitingTime || energy > 500000) {
                droneTripList[index].emplace_back();
                travelTime[i] = time[0][nextCus];
            } else {
                travelTime[i] += time[lastCus][nextCus];
            }
            droneTripList[index].back().push_back(nextCus);

            visitedCus[nextCus] = true;
            numVisitedCus++;
        } else {
            time = input.techTimes;
            index -= config.numDrone;

            if (techTripList[index].empty()) {
                lastCus = 0;
            } else {
                lastCus = techTripList[index].back();
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

            double timeGoToFirstCus = time[0][nextCus];
            if (!techTripList[index].empty()) {
                timeGoToFirstCus = time[0][techTripList[index][0]];
            }

            double waitTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0] - timeGoToFirstCus;

            if (waitTime <= config.sampleLimitationWaitingTime) {
                techTripList[index].push_back(nextCus);
                travelTime[i] += time[lastCus][nextCus];
                visitedCus[nextCus] = true;
                numVisitedCus++;
            }
        }

        i++;
        i %= config.numDrone + config.numTech;
    }
    // std::cout << " \n Run + 1" << std::endl;
    // std::cout << droneTripList.size() << std::endl;
    // for(int i = 0; i < droneTripList.size(); i ++){
    //     // std::cout << "Trip : ";
    //     for (int  j = 0; j < droneTripList[i].size(); j++){
    //         for (int k = 0; k < droneTripList[i][j].size(); k++){
    //             std::cout << std::setw(5) << droneTripList.at(i).at(j).at(k);
    //         }
            
    //     }
        
    // }
    // std::cout << "\n";
}

void Solution::initByAngle(bool reverse, int direction) {
    std::vector<std::vector<int>> orderDistance(input.numCus + 1);

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
            double timeGoToFirstCus = time[0][orderAngle[0]];
            int remainCus = (int) orderAngle.size();
            while (j < remainCus) {
                int nextCus = orderAngle[j];
                if (!input.cusOnlyServedByTech[nextCus]) {
                    double flightTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0];
                    double waitTime = flightTime - timeGoToFirstCus;

                    if (flightTime <= config.droneLimitationFightTime &&
                        waitTime <= config.sampleLimitationWaitingTime) {
                        travelTime[i] += time[lastCus][nextCus];
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
            if (techTripList[index].empty()) {
                time = input.techTimes;
                int j = 0;
                lastCus = 0;
                double timeGoToFirstCus = time[0][orderAngle[0]];
                int remainCus = (int) orderAngle.size();
                while (j < remainCus) {
                    int nextCus = orderAngle[j];
                    double waitTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0] - timeGoToFirstCus;

                    if (waitTime <= config.sampleLimitationWaitingTime) {
                        techTripList[index].push_back(nextCus);
                        travelTime[i] += time[lastCus][nextCus];
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
        }

        i++;
        i %= config.numDrone + config.numTech;
    }
}

double Solution::getScore() {
    std::vector<double> techCompleteTime(config.numTech, 0);
    std::vector<double> droneCompleteTime(config.numDrone, 0);
    std::vector<double> cusCompleteTime(input.numCus + 1, 0);
    std::vector<std::vector<double>> droneTripCompleteTime;

    int maxTrip = 0;
    for (auto &i: droneTripList) {
        if (maxTrip < i.size()) {
            maxTrip = (int) i.size();
        }
    }

    for (int i = 0; i < config.numDrone; i++) {
        std::vector<double> tripTime(maxTrip, 0);
        droneTripCompleteTime.push_back(tripTime);
    }

    double tmp, tmp1;
    dz = 0, cz = 0;

    double allTechTime = 0, allDroneTime = 0;

    for (int i = 0; i < techTripList.size(); i++) {
        if (techTripList[i].empty()) {
            continue;
        }

        tmp = input.techTimes[0][techTripList[i][0]];

        cusCompleteTime[techTripList[i][0]] = tmp;

        for (int j = 0; j < (int) techTripList[i].size() - 1; j++) {
            tmp += input.techTimes[techTripList[i][j]][techTripList[i][j + 1]];
            cusCompleteTime[techTripList[i][j + 1]] = tmp;
        }

        techCompleteTime[i] = tmp + input.techTimes[techTripList[i].back()][0];
        if (techCompleteTime[i] > allTechTime) {
            allTechTime = techCompleteTime[i];
        }
    }

    for (int i = 0; i < droneTripList.size(); i++) {
        tmp = 0;
        for (int j = 0; j < droneTripList[i].size(); j++) {
            if (droneTripList[i][j].empty()) {
                continue;
            }

            tmp1 = input.droneTimes[0][droneTripList[i][j][0]];
            cusCompleteTime[droneTripList[i][j][0]] = tmp1;

            for (int k = 0; k < (int) droneTripList[i][j].size() - 1; k++) {
                tmp1 += input.droneTimes[droneTripList[i][j][k]][droneTripList[i][j][k + 1]];
                cusCompleteTime[droneTripList[i][j][k + 1]] = tmp1;
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

    for (int i = 0; i < techTripList.size(); i++) {
        if (techTripList[i].empty()) { continue; }

        for (int j: techTripList[i]) {
            cz += std::max(0., techCompleteTime[i] - cusCompleteTime[j] - config.sampleLimitationWaitingTime);
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
            dz += std::max(0., droneTripCompleteTime[i][j] - config.droneLimitationFightTime);
        }
    }

    return c + alpha1 * cz + alpha2 * dz;
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

Solution *Solution::relocate(const std::vector<std::string> &tabuList, double bestScore, RouteType type) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

    if (type != INTER) {
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

                        double newScore = s.getScore();
                        if (newScore < curScore)
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                    s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
                                isImproved = true;
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

                        double newScore = s.getScore();

                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                    s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
                                isImproved = true;
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

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++) {

                if (xIndex != 0) {
                    Solution s = *this;

                    s.techTripList[techIndex].insert(s.techTripList[techIndex].begin(),
                                                     s.techTripList[techIndex][xIndex]);


                    s.techTripList[techIndex].erase(s.techTripList[techIndex].begin() + xIndex + 1);

                    double newScore = s.getScore();

                    if (newScore < curScore) {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                s.techTripList[techIndex][xIndex])) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon)) {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                    }
                }

                for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++) {
                    if (yIndex == xIndex || yIndex == xIndex - 1) {
                        continue;
                    }

                    Solution s = *this;

                    s.techTripList[techIndex].insert(s.techTripList[techIndex].begin() + yIndex + 1,
                                                     s.techTripList[techIndex][xIndex]);

                    if (xIndex < yIndex) {
                        s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex);
                    } else {
                        s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex + 1);
                    }

                    double newScore = s.getScore();

                    if (newScore < curScore) {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                s.techTripList[techIndex][xIndex])) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon)) {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                    }
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

                            double newScore = s.getScore();

                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                        s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                            droneTripList[droneIndex][tripIndex][xIndex]);

                                    curScore = newScore;
                                }
                            }

                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                                s = *this;
                                s.droneTripList[droneIndex2][tripIndex2].insert(
                                        s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1,
                                        s.droneTripList[droneIndex][tripIndex][xIndex]);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                                newScore = s.getScore();

                                if (newScore < curScore) {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                            s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
                                        isImproved = true;
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

                    //tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        Solution s = *this;

                        s.techTripList[techIndex].insert(s.techTripList[techIndex].begin(),
                                                         s.droneTripList[droneIndex][tripIndex][xIndex]);

                        s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                        double newScore = s.getScore();

                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                    s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);

                                curScore = newScore;
                            }
                        }

                        for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++) {
                            s = *this;

                            s.techTripList[techIndex].insert(s.techTripList[techIndex].begin() + yIndex + 1,
                                                             s.droneTripList[droneIndex][tripIndex][xIndex]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                            newScore = s.getScore();

                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                        s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
                                    isImproved = true;
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
            for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++) {

                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                    if (techIndex == techIndex2) {
                        continue;
                    }

                    Solution s = *this;

                    s.techTripList[techIndex2].insert(s.techTripList[techIndex2].begin(),
                                                      s.techTripList[techIndex][xIndex]);

                    s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex);


                    double newScore = s.getScore();

                    if (newScore < curScore) {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                s.techTripList[techIndex][xIndex])) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon)) {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                    }

                    for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++) {
                        Solution s = *this;

                        s.techTripList[techIndex2].insert(s.techTripList[techIndex2].begin() + yIndex + 1,
                                                          s.techTripList[techIndex][xIndex]);

                        s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex);


                        double newScore = s.getScore();

                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                    s.techTripList[techIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                                curScore = newScore;
                            }
                        }
                    }
                }

                // drone
                if (input.cusOnlyServedByTech[techTripList[techIndex][xIndex]]) {
                    continue;
                }

                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                        Solution s = *this;

                        s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin(),
                                s.techTripList[techIndex][xIndex]);

                        s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex);


                        double newScore = s.getScore();

                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                    s.techTripList[techIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                                curScore = newScore;
                            }
                        }

                        for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                            Solution s = *this;

                            s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin() + yIndex + 1,
                                    s.techTripList[techIndex][xIndex]);

                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + xIndex);


                            double newScore = s.getScore();

                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                        s.techTripList[techIndex][xIndex])) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
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

Solution *Solution::exchange(const std::vector<std::string> &tabuList, double bestScore, RouteType type) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

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
                                double newScore = s.getScore();

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                std::string val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                if (newScore < curScore) {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                    //tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++) {
                            Solution s = *this;

                            std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                      s.techTripList[techIndex][yIndex]);

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            std::string val2 = std::to_string(techTripList[techIndex][yIndex]);

                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++) {

                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                    if (techIndex == techIndex2) {
                        continue;
                    }
                    for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++) {
                        Solution s = *this;

                        std::swap(s.techTripList[techIndex][xIndex], s.techTripList[techIndex2][yIndex]);

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                        std::string val2 = std::to_string(techTripList[techIndex2][yIndex]);

                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                // drone
                if (input.cusOnlyServedByTech[techTripList[techIndex][xIndex]]) {
                    continue;
                }

                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                        for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                            Solution s = *this;

                            std::swap(s.techTripList[techIndex][xIndex],
                                      s.droneTripList[droneIndex][tripIndex][yIndex]);

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);

                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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

    if (type != INTER) {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < (int) droneTripList[droneIndex][tripIndex].size() - 1; xIndex++) {
                    for (int yIndex = xIndex + 1; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                        Solution s = *this;

                        std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                  s.droneTripList[droneIndex][tripIndex][yIndex]);

                        double newScore = s.getScore();
                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);

                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            for (int xIndex = 0; xIndex < (int) techTripList[techIndex].size() - 1; xIndex++) {
                for (int yIndex = xIndex + 1; yIndex < techTripList[techIndex].size(); yIndex++) {
                    Solution s = *this;

                    std::swap(s.techTripList[techIndex][xIndex],
                              s.techTripList[techIndex][yIndex]);

                    double newScore = s.getScore();
                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][yIndex]);

                    if (newScore < curScore) {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon)) {
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

    if (isImproved) {
        return bestSolution;
    } else {
        return nullptr;
    }
}

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

Solution *Solution::orOpt(const std::vector<std::string> &tabuList, double bestScore, RouteType type, int dis) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

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

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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
                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++) {
                                s = *this;
                                for (int i = 0; i <= dis; i++) {
                                    s.droneTripList[droneIndex2][tripIndex2].insert(
                                            s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + i + 1,
                                            droneTripList[droneIndex][tripIndex][xIndex + i]);

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                            s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                                }

                                newScore = s.getScore();

                                val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                                if (newScore < curScore) {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
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
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        Solution s = *this;

                        for (int i = dis; i >= 0; i--) {
                            s.techTripList[techIndex].insert(
                                    s.techTripList[techIndex].begin(),
                                    droneTripList[droneIndex][tripIndex][xIndex + i]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + i);
                        }

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                        for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++) {
                            s = *this;
                            for (int i = 0; i <= dis; i++) {
                                s.techTripList[techIndex].insert(
                                        s.techTripList[techIndex].begin() + yIndex + i + 1,
                                        droneTripList[droneIndex][tripIndex][xIndex + i]);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                            }

                            newScore = s.getScore();

                            val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            if (techTripList[techIndex].size() <= dis) {
                continue;
            }
            for (int xIndex = 0; xIndex < (int) techTripList[techIndex].size() - dis; xIndex++) {
                //tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                    if (techIndex == techIndex2) {
                        continue;
                    }

                    Solution s = *this;
                    for (int i = dis; i >= 0; i--) {
                        s.techTripList[techIndex2].insert(
                                s.techTripList[techIndex2].begin(),
                                techTripList[techIndex][xIndex + i]);

                        s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex + i);
                    }

                    double newScore = s.getScore();

                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                    if (newScore < curScore) {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                    for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++) {
                        s = *this;
                        for (int i = 0; i <= dis; i++) {
                            s.techTripList[techIndex2].insert(
                                    s.techTripList[techIndex2].begin() + yIndex + i + 1,
                                    techTripList[techIndex][xIndex + i]);

                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + xIndex);
                        }

                        newScore = s.getScore();

                        val1 = std::to_string(techTripList[techIndex][xIndex]);
                        val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                //drone
                bool canMoveToDroneTrip = true;
                for (int i = xIndex; i < xIndex + dis; i++) {
                    if (input.cusOnlyServedByTech[techTripList[techIndex][i]]) {
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
                                    techTripList[techIndex][xIndex + i]);

                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + xIndex + i);
                        }

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                        std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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
                        for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++) {
                            s = *this;
                            for (int i = 0; i <= dis; i++) {
                                s.droneTripList[droneIndex][tripIndex].insert(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex + i + 1,
                                        techTripList[techIndex][xIndex + i]);

                                s.techTripList[techIndex].erase(
                                        s.techTripList[techIndex].begin() + xIndex);
                            }

                            newScore = s.getScore();

                            val1 = std::to_string(techTripList[techIndex][xIndex]);
                            val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
            if (techTripList[techIndex].size() <= dis) {
                continue;
            }
            for (int xIndex = 0; xIndex < (int) techTripList[techIndex].size() - dis; xIndex++) {
                if (xIndex != 0) {
                    Solution s = *this;
                    for (int i = dis; i >= 0; i--) {
                        s.techTripList[techIndex].insert(
                                s.techTripList[techIndex].begin(),
                                techTripList[techIndex][xIndex + i]);

                        s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex + dis + 1);
                    }

                    double newScore = s.getScore();

                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                    if (newScore < curScore) {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++) {
                    if (yIndex == xIndex || (yIndex < xIndex && yIndex + dis >= xIndex)) {
                        continue;
                    }

                    Solution s = *this;
                    for (int i = 0; i <= dis; i++) {
                        s.techTripList[techIndex].insert(
                                s.techTripList[techIndex].begin() + yIndex + i + 1,
                                techTripList[techIndex][xIndex + i]);
                    }

                    for (int i = 0; i <= dis; i++) {
                        if (xIndex < yIndex) {
                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + xIndex);
                        } else {
                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + xIndex + dis + 1);
                        }
                    }
                    double newScore = s.getScore();

                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                    if (newScore < curScore) {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon)) {
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

    if (isImproved) {
        return bestSolution;
    } else {
        return nullptr;
    }
}

Solution *Solution::twoOpt(const std::vector<std::string> &tabuList, double bestScore, RouteType type) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

    if (type != INTRA) {
        // drone
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = -1; xIndex < (int) droneTripList[droneIndex][tripIndex].size(); xIndex++) {
                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for (int yIndex = -1; yIndex < (int) techTripList[techIndex].size(); yIndex++) {
                            std::vector<int> tmp1, tmp2;
                            if (xIndex >= 0) {
                                tmp1.insert(tmp1.end(),
                                            droneTripList[droneIndex][tripIndex].begin(),
                                            droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                            }

                            if (yIndex >= 0) {
                                tmp2.insert(tmp2.end(),
                                            techTripList[techIndex].begin(),
                                            techTripList[techIndex].begin() + yIndex + 1);
                            }

                            if (yIndex < (int) techTripList[techIndex].size()) {
                                tmp1.insert(tmp1.end(),
                                            techTripList[techIndex].begin() + yIndex + 1,
                                            techTripList[techIndex].end());
                            }

                            if (xIndex < (int) droneTripList[droneIndex][tripIndex].size()) {
                                tmp2.insert(tmp2.end(),
                                            droneTripList[droneIndex][tripIndex].begin() + xIndex + 1,
                                            droneTripList[droneIndex][tripIndex].end());
                            }
                            Solution s = *this;

                            s.droneTripList[droneIndex][tripIndex] = tmp1;
                            s.techTripList[techIndex] = tmp2;

                            double newScore = s.getScore();

                            std::string val1 = "0";
                            if (xIndex >= 0) {
                                val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            }
                            std::string val2 = "0";
                            if (yIndex >= 0) {
                                val2 = std::to_string(techTripList[techIndex][yIndex]);
                            }
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                                double newScore = s.getScore();
                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                }
                                if (newScore < curScore) {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
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
            for (int xIndex = -1; xIndex < (int) techTripList[techIndex].size(); xIndex++) {
                // drone
                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                        for (int yIndex = -1; yIndex < (int) droneTripList[droneIndex][tripIndex].size(); yIndex++) {

                            bool canMoveToDroneTrip = true;
                            for (int i = xIndex + 1; i < techTripList[techIndex].size(); i++) {
                                if (input.cusOnlyServedByTech[techTripList[techIndex][i]]) {
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
                                            techTripList[techIndex].begin(),
                                            techTripList[techIndex].begin() + xIndex + 1);
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

                            if (xIndex < (int) techTripList[techIndex].size()) {
                                tmp2.insert(tmp2.end(),
                                            techTripList[techIndex].begin() + xIndex + 1,
                                            techTripList[techIndex].end());
                            }
                            Solution s = *this;

                            s.techTripList[techIndex] = tmp1;
                            s.droneTripList[droneIndex][tripIndex] = tmp2;

                            double newScore = s.getScore();
                            std::string val1 = "0";
                            if (xIndex >= 0) {
                                val1 = std::to_string(techTripList[techIndex][xIndex]);
                            }
                            std::string val2 = "0";
                            if (yIndex >= 0) {
                                val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);
                            }
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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
                    if (techIndex == techIndex2) {
                        continue;
                    }
                    for (int yIndex = -1; yIndex < (int) techTripList[techIndex2].size(); yIndex++) {
                        std::vector<int> tmp1, tmp2;
                        if (xIndex >= 0) {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex].begin(),
                                        techTripList[techIndex].begin() + xIndex + 1);
                        }

                        if (yIndex >= 0) {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex2].begin(),
                                        techTripList[techIndex2].begin() + yIndex + 1);
                        }

                        if (yIndex < (int) techTripList[techIndex2].size()) {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex2].begin() + yIndex + 1,
                                        techTripList[techIndex2].end());
                        }

                        if (xIndex < (int) techTripList[techIndex].size()) {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex].begin() + xIndex + 1,
                                        techTripList[techIndex].end());
                        }
                        Solution s = *this;

                        s.techTripList[techIndex] = tmp1;
                        s.techTripList[techIndex2] = tmp2;

                        double newScore = s.getScore();
                        std::string val1 = "0";
                        if (xIndex >= 0) {
                            val1 = std::to_string(techTripList[techIndex][xIndex]);
                        }
                        std::string val2 = "0";
                        if (yIndex >= 0) {
                            val2 = std::to_string(techTripList[techIndex2][yIndex]);
                        }
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                                double newScore = s.getScore();
                                std::string val1 = "0";
                                if (xIndex >= 0) {
                                    val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0) {
                                    val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                }
                                if (newScore < curScore) {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
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
            for (int xIndex = -1; xIndex < (int) techTripList[techIndex].size(); xIndex++) {
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                    if (techIndex == techIndex2) {
                        continue;
                    }
                    for (int yIndex = -1; yIndex < (int) techTripList[techIndex2].size(); yIndex++) {
                        std::vector<int> tmp1, tmp2;
                        if (xIndex >= 0) {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex].begin(),
                                        techTripList[techIndex].begin() + xIndex + 1);
                        }

                        if (yIndex >= 0) {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex2].begin(),
                                        techTripList[techIndex2].begin() + yIndex + 1);
                        }

                        if (yIndex < (int) techTripList[techIndex2].size()) {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex2].begin() + yIndex + 1,
                                        techTripList[techIndex2].end());
                        }

                        if (xIndex < (int) techTripList[techIndex].size()) {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex].begin() + xIndex + 1,
                                        techTripList[techIndex].end());
                        }
                        Solution s = *this;

                        s.techTripList[techIndex] = tmp1;
                        s.techTripList[techIndex2] = tmp2;

                        double newScore = s.getScore();
                        std::string val1 = "0";
                        if (xIndex >= 0) {
                            val1 = std::to_string(techTripList[techIndex][xIndex]);
                        }
                        std::string val2 = "0";
                        if (yIndex >= 0) {
                            val2 = std::to_string(techTripList[techIndex2][yIndex]);
                        }
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

    if (isImproved) {
        return bestSolution;
    } else {
        return nullptr;
    }
}

Solution *Solution::crossExchange(const std::vector<std::string> &tabuList, double bestScore, RouteType type, int dis1,
                                  int dis2) {
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;
    if (type != INTRA) {
        // drone

        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                for (int xIndex = 0; xIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis1; xIndex++) {
                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
                        for (int yIndex = 0; yIndex < (int) techTripList[techIndex].size() - dis2; yIndex++) {
                            bool canMoveToDroneTrip = true;
                            for (int i = yIndex; i <= yIndex + dis2; i++) {
                                if (input.cusOnlyServedByTech[techTripList[techIndex][i]]) {
                                    canMoveToDroneTrip = false;
                                    break;
                                }
                            }
                            if (!canMoveToDroneTrip) {
                                continue;
                            }

                            Solution s = *this;

                            s.techTripList[techIndex].insert(
                                    s.techTripList[techIndex].begin() + yIndex + dis2 + 1,
                                    droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                    droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1
                            );
                            s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1,
                                    techTripList[techIndex].begin() + yIndex,
                                    techTripList[techIndex].begin() + yIndex + dis2 + 1
                            );

                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + yIndex,
                                    s.techTripList[techIndex].begin() + yIndex + dis2 + 1
                            );

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1
                            );

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex])
                                               + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(techTripList[techIndex][yIndex])
                                               + "-" + std::to_string(techTripList[techIndex][yIndex + dis2]);
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                                double newScore = s.getScore();

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex])
                                                   + "-" +
                                                   std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                                std::string val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex])
                                                   + "-" + std::to_string(
                                        droneTripList[droneIndex2][tripIndex2][yIndex + dis2]);
                                if (newScore < curScore) {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon)) {
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
            for (int xIndex = 0; xIndex < (int) techTripList[techIndex].size() - dis1; xIndex++) {

                // drone
                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++) {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++) {
                        for (int yIndex = 0;
                             yIndex < (int) droneTripList[droneIndex][tripIndex].size() - dis2; yIndex++) {
                            bool canMoveToDroneTrip = true;
                            for (int i = xIndex; i <= xIndex + dis1; i++) {
                                if (input.cusOnlyServedByTech[techTripList[techIndex][i]]) {
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
                                    techTripList[techIndex].begin() + xIndex,
                                    techTripList[techIndex].begin() + xIndex + dis1 + 1
                            );
                            s.techTripList[techIndex].insert(
                                    s.techTripList[techIndex].begin() + xIndex + dis1 + 1,
                                    droneTripList[droneIndex][tripIndex].begin() + yIndex,
                                    droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1
                            );

                            s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + yIndex,
                                    s.droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1
                            );

                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + xIndex,
                                    s.techTripList[techIndex].begin() + xIndex + dis1 + 1
                            );

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(techTripList[techIndex][xIndex])
                                               + "-" + std::to_string(techTripList[techIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex])
                                               + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][yIndex + dis2]);
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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
                    for (int yIndex = 0; yIndex < (int) techTripList[techIndex2].size() - dis2; yIndex++) {
                        if (techIndex == techIndex2) {
                            continue;
                        }
                        Solution s = *this;

                        s.techTripList[techIndex2].insert(
                                s.techTripList[techIndex2].begin() + yIndex + dis2 + 1,
                                techTripList[techIndex].begin() + xIndex,
                                techTripList[techIndex].begin() + xIndex + dis1 + 1
                        );
                        s.techTripList[techIndex].insert(
                                s.techTripList[techIndex].begin() + xIndex + dis1 + 1,
                                techTripList[techIndex2].begin() + yIndex,
                                techTripList[techIndex2].begin() + yIndex + dis2 + 1
                        );

                        s.techTripList[techIndex2].erase(
                                s.techTripList[techIndex2].begin() + yIndex,
                                s.techTripList[techIndex2].begin() + yIndex + dis2 + 1
                        );

                        s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex,
                                s.techTripList[techIndex].begin() + xIndex + dis1 + 1
                        );

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex])
                                           + "-" + std::to_string(techTripList[techIndex][xIndex + dis1]);
                        std::string val2 = std::to_string(techTripList[techIndex2][yIndex])
                                           + "-" +
                                           std::to_string(techTripList[techIndex2][yIndex + dis2]);
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex])
                                               + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex])
                                               + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][yIndex + dis2]);
                            if (newScore < curScore) {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon)) {
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
            for (int xIndex = 0; xIndex < (int) techTripList[techIndex].size() - dis1; xIndex++) {
                for (int yIndex = 0; yIndex < (int) techTripList[techIndex].size() - dis2; yIndex++) {
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
                                   techTripList[techIndex].begin(),
                                   techTripList[techIndex].begin() + beg1);
                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + beg2,
                                   techTripList[techIndex].begin() + end2);
                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + end1,
                                   techTripList[techIndex].begin() + beg2);
                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + beg1,
                                   techTripList[techIndex].begin() + end1);

                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + end2,
                                   techTripList[techIndex].end());

                        s.techTripList[techIndex].erase(s.techTripList[techIndex].begin(),
                                                        s.techTripList[techIndex].end());
                        s.techTripList[techIndex].insert(s.techTripList[techIndex].begin(),
                                                         tmp.begin(), tmp.end());

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex])
                                           + "-" + std::to_string(techTripList[techIndex][xIndex + dis1]);
                        std::string val2 = std::to_string(techTripList[techIndex][yIndex])
                                           + "-" +
                                           std::to_string(techTripList[techIndex][yIndex + dis2]);
                        if (newScore < curScore) {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon)) {
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
        for (int xIndex = 0; xIndex < s.techTripList[techIndex].size(); xIndex++) {
            ejection(s,
                     {techIndex, xIndex},
                     TECHNICIAN,
                     currentGain,
                     bestGain,
                     currentLevel,
                     shiftSequence,
                     bestShiftSequence);
        }
    }

    for (std::pair<std::vector<int>, std::vector<int>> move: bestShiftSequence) {
        int x;
        if (move.first.size() == 2) {
            x = s.techTripList[move.first[0]][move.first[1]];
            s.techTripList[move.first[0]].erase(s.techTripList[move.first[0]].begin() + move.first[1]);
        } else {
            x = s.droneTripList[move.first[0]][move.first[1]][move.first[2]];
            s.droneTripList[move.first[0]][move.first[1]].erase(
                    s.droneTripList[move.first[0]][move.first[1]].begin() + move.first[2]);
        }

        if (move.second.size() == 2) {
            s.techTripList[move.second[0]].insert(s.techTripList[move.second[0]].begin() + move.second[1], x);
        } else {
            s.droneTripList[move.second[0]][move.second[1]].insert(
                    s.droneTripList[move.second[0]][move.second[1]].begin() + move.second[2], x);
        }
    }
//    std::cout<< "-gain:" << bestGain<<std::endl;
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
        std::vector<double> techCompleteTime(config.numTech, 0);
        if (!techTripList[tripIndex].empty()) {
            tmp = input.techTimes[0][techTripList[tripIndex][0]];

            cusCompleteTime[techTripList[tripIndex][0]] = tmp;

            for (int j = 0; j < (int) techTripList[tripIndex].size() - 1; j++) {
                tmp += input.techTimes[techTripList[tripIndex][j]][techTripList[tripIndex][j + 1]];
                cusCompleteTime[techTripList[tripIndex][j + 1]] = tmp;
            }

            techCompleteTime[tripIndex] = tmp + input.techTimes[techTripList[tripIndex].back()][0];
            if (techCompleteTime[tripIndex] > allTechTime) {
                allTechTime = techCompleteTime[tripIndex];
            }

            for (int j: techTripList[tripIndex]) {
                cz += std::max(0.,
                               techCompleteTime[tripIndex] - cusCompleteTime[j] - config.sampleLimitationWaitingTime);
            }

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
            for (int cusIndex = 0; cusIndex < solution.techTripList[techIndex].size(); cusIndex++) {
                int cus = solution.techTripList[techIndex][cusIndex];
                int cusPredecessor = (cusIndex == 0) ? 0 : solution.techTripList[techIndex][cusIndex - 1];
                double delta = input.techTimes[cusPredecessor][x]
                               + input.techTimes[x][cus] - input.techTimes[cusPredecessor][cus];
                double d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                if (gain - d > bestGain) {
                    solution.techTripList[techIndex].insert(
                            solution.techTripList[techIndex].begin() + cusIndex, x);
                    gain -= d;

                    shiftSequence.push_back({xIndex, {techIndex, cusIndex}});
                    level++;
                    std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                    if (tmp[1] == 0 && tmp[2] == 0) {
                        bestShiftSequence = shiftSequence;
                        bestGain = gain;
                    } else if (level + 1 <= config.maxEjectionLevel) {
                        for (int yIndex = 0;
                             yIndex < solution.techTripList[techIndex].size(); yIndex++) {
                            Solution s = solution;

                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + yIndex);
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

                    solution.techTripList[techIndex].erase(
                            solution.techTripList[techIndex].begin() + cusIndex);
                    gain += d;
                    shiftSequence.pop_back();
                    level--;
                }

                if (cusIndex == solution.techTripList[techIndex].size() - 1) {
                    delta = input.techTimes[cus][x]
                            + input.techTimes[x][input.numCus + 1] - input.techTimes[cus][input.numCus + 1];
                    d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                    if (gain - d > bestGain) {
                        solution.techTripList[techIndex].insert(
                                solution.techTripList[techIndex].begin() + cusIndex + 1, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {techIndex, cusIndex + 1}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                        if (tmp[1] == 0 && tmp[2] == 0) {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        } else if (level + 1 <= config.maxEjectionLevel) {
                            for (int yIndex = 0;
                                 yIndex < solution.techTripList[techIndex].size(); yIndex++) {
                                Solution s = solution;

                                s.techTripList[techIndex].erase(
                                        s.techTripList[techIndex].begin() + yIndex);
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

                        solution.techTripList[techIndex].erase(
                                solution.techTripList[techIndex].begin() + cusIndex);
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }
                }
            }
        }

        solution.droneTripList[xIndex[0]][xIndex[1]].insert(
                solution.droneTripList[xIndex[0]][xIndex[1]].begin() + xIndex[2], x);
    } else {
        times = input.techTimes;
        x = solution.techTripList[xIndex[0]][xIndex[1]];
        predecessor = (xIndex[1] == 0) ? 0 : solution.techTripList[xIndex[0]][xIndex[1] - 1];
        successor = (xIndex[1] == solution.techTripList[xIndex[0]].size() - 1)
                    ? input.numCus + 1
                    : solution.techTripList[xIndex[0]][xIndex[1] + 1];
        techScores[xIndex[0]] -= times[predecessor][x] + times[x][successor] - times[predecessor][successor];

        double cScore = std::max(maxDroneScore, *std::max_element(techScores.begin(), techScores.end()));
        double g = fScore - cScore;
        solution.techTripList[xIndex[0]].erase(
                solution.techTripList[xIndex[0]].begin() + xIndex[1]);
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
            if (techIndex == xIndex[0]) {
                continue;
            }
            for (int cusIndex = 0; cusIndex < solution.techTripList[techIndex].size(); cusIndex++) {
                int cus = solution.techTripList[techIndex][cusIndex];
                int cusPredecessor = (cusIndex == 0) ? 0 : solution.techTripList[techIndex][cusIndex - 1];
                double delta = input.techTimes[cusPredecessor][x]
                               + input.techTimes[x][cus] - input.techTimes[cusPredecessor][cus];
                double d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                if (gain - d > bestGain) {
                    solution.techTripList[techIndex].insert(
                            solution.techTripList[techIndex].begin() + cusIndex, x);
                    gain -= d;

                    shiftSequence.push_back({xIndex, {techIndex, cusIndex}});
                    level++;
                    std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                    if (tmp[1] == 0 && tmp[2] == 0) {
                        bestShiftSequence = shiftSequence;
                        bestGain = gain;
                    } else if (level + 1 <= config.maxEjectionLevel) {
                        for (int yIndex = 0;
                             yIndex < solution.techTripList[techIndex].size(); yIndex++) {
                            Solution s = solution;

                            s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + yIndex);
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

                    solution.techTripList[techIndex].erase(
                            solution.techTripList[techIndex].begin() + cusIndex);
                    gain += d;
                    shiftSequence.pop_back();
                    level--;
                }

                if (cusIndex == solution.techTripList[techIndex].size() - 1) {
                    delta = input.techTimes[cus][x]
                            + input.techTimes[x][input.numCus + 1] - input.techTimes[cus][input.numCus + 1];
                    d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                    if (gain - d > bestGain) {
                        solution.techTripList[techIndex].insert(
                                solution.techTripList[techIndex].begin() + cusIndex + 1, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {techIndex, cusIndex + 1}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                        if (tmp[1] == 0 && tmp[2] == 0) {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        } else if (level + 1 <= config.maxEjectionLevel) {
                            for (int yIndex = 0;
                                 yIndex < solution.techTripList[techIndex].size(); yIndex++) {
                                Solution s = solution;

                                s.techTripList[techIndex].erase(
                                        s.techTripList[techIndex].begin() + yIndex);
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

                        solution.techTripList[techIndex].erase(
                                solution.techTripList[techIndex].begin() + cusIndex);
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }
                }
            }
        }

        solution.techTripList[xIndex[0]].insert(
                solution.techTripList[xIndex[0]].begin() + xIndex[1], x);
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
                    for (int &yIndex: techIndex) {
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

    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++) {
        for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++) {

            // tech
            for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++) {
                if (techIndex == techIndex2) {
                    continue;
                }
                for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++) {
                    if (Random::get(0.0, 1.0) > swapRate) {
                        continue;
                    }

                    std::swap(techTripList[techIndex][xIndex], techTripList[techIndex2][yIndex]);
                    num++;
                }
            }

            // drone
            if (input.cusOnlyServedByTech[techTripList[techIndex][xIndex]]) {
                continue;
            }

            for (auto &droneIndex: droneTripList) {
                for (auto &tripIndex: droneIndex) {
                    for (int &yIndex: tripIndex) {
                        if (Random::get(0.0, 1.0) > swapRate) {
                            continue;
                        }

                        std::swap(techTripList[techIndex][xIndex],
                                  yIndex);
                        num++;
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
    std::cout << " \nDrone Trip Init : " << droneTripList.size() << std::endl;
    for(auto trips : droneTripList){
        std::cout << "\n";
        for(auto trip : trips){
            for (auto x : trip){
                std::cout <<" ||" << x ;
            }
            
            double energy = check_Capacity(trip, input);
            std::cout << std::setprecision(20);
            std::cout << " \nTrip energy : " << energy << "\n";
        }
    }
    std::cout << " \nTech Trip Init : " << techTripList.size() << std::endl;
    for(auto techTrips : techTripList){
        std::cout << "\n";
        for(auto techTrip : techTrips){
            std:: cout << "||" << techTrip;
        }
    }
    std::cout << " \n "  << std::endl;
}

Solution::Solution() = default;




