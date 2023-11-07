#pragma comment(linker, "/STACK:2000000")
#include "Score.h"
#include "math.h"
#include <chrono>
#include "iostream"
using namespace std::chrono;

Config config2;

double _k1 = config2.k1;
double _k2 = config2.k2;
double _c1 = config2.c1;
double _c2 = config2.c2;
double _c4 = config2.c4;
double _c5 = config2.c5;
double _g = config2.g; // gravitational constant;
double _alpha = config2.alpha; // angle off attack;
double _takeoff_speed = config2.droneTakeoffSpeed;
double _landing_speed = config2.droneLandingSpeed;
double _drone_speed = config2.droneVelocity;
double _takeoff_time = config2.cruiseAlt / config2.droneTakeoffSpeed;
double _landing_time = config2.cruiseAlt / config2.droneLandingSpeed;
double _W = config2.W;

double landing_Takeoff(double demand, double speed){
    double P;
    P = _k1*(_W + demand)*_g*(speed/2 + sqrt(pow((speed/2), 2.) + (_W + demand)*_g/pow(_k2, 2.))) + _c2*sqrt(pow(((_W + demand)*_g), 3.));
    return P;
}

double horizontal1(double demand, double speed){
    double P;
    double P1 = pow(((_W + demand)*_g - _c5*pow((speed*cos(_alpha)), 2.)), 2.);
    double P2 = pow(_c4*pow(speed, 2.), 2.);
    double P3 = _c4*pow(speed, 3);
    double P4 = pow(speed, 3.);
    P = (_c1 + _c2)*pow((P1 + P2 ), 3/4.) + P3;
    return P;
}

double Score::countEnergy(int cus1, int cus2, double demand, Input &input){
    double time = input.droneTimes[cus1][cus2];
    double total_Energy = 0;
    total_Energy += landing_Takeoff(demand, _takeoff_speed) * _takeoff_time;
    total_Energy += horizontal1(demand, _drone_speed) * time ; 
    total_Energy += landing_Takeoff(demand, _landing_speed) * _landing_time;
    return total_Energy;
}

int Score::check_ratio1(double time, Input &input){
    int i;
    for (i = 0; i < input.ratio_v.size(); i++){
        if (i * 3600 <= time && time < (i + 1) * 3600){
            break;
        }
    }
    return i;
}

double Score::countTimeTruck1(double startTime, double distance, Input &input){ 
    double t = startTime;
    double round = 0;
    while(t > 12 * 3600){
        t = t - 12 * 3600;
        round ++;
    }
    int i = check_ratio1(startTime, input);
    double time;
    time = t + distance/(input.truckV_max*input.ratio_v[i]);
    while (time > (i+1) * 3600){
        distance = distance - input.truckV_max * input.ratio_v[i] * ((i+1) * 3600 - t);
        t = (i+1) * 3600;
        if (t == 12 * 3600){
            round++;
            i =  -1;
            t = 0;
        }
        time = t + distance/(input.truckV_max*input.ratio_v[i+1]);
        i++;
    }
    return ( time + round*12 *3600 - startTime);  
}

void Score::updateDroneScore(std::vector<std::vector<std::vector<int>>> droneTripList, Score &score, Input &input, int droneIndex, int tripIndex, int index){
    std::vector<double> cusCompleteTime(input.numCus + 1, 0);
    double newWaitingTime = 0;

    if (droneTripList[droneIndex][tripIndex].empty()){
        score.droneDemand[droneIndex][tripIndex] = 0;
        score.droneTime[droneIndex][tripIndex] = 0;
        score.droneWaitingTime[droneIndex][tripIndex] = 0;
        score.droneOverEnergy[droneIndex][tripIndex] = 0;
        while (!score.droneEnergy[droneIndex][tripIndex].empty()){
            score.droneEnergy[droneIndex][tripIndex].pop_back();
        }
    }else{
        //update demand of drone
        double newDemand = input.demand[droneTripList[droneIndex][tripIndex][0]];
        double newDroneTime = input.droneTimes[0][droneTripList[droneIndex][tripIndex][0]] + input.serviceTimeByDrone[droneTripList[droneIndex][tripIndex][0]] + _takeoff_time + _landing_time;
        cusCompleteTime[droneTripList[droneIndex][tripIndex][0]] = newDroneTime;
        for (int i = 0; i < droneTripList[droneIndex][tripIndex].size() - 1; i++ ){
            newDemand +=  input.demand[droneTripList[droneIndex][tripIndex][i + 1]];
            newDroneTime += input.droneTimes[droneTripList[droneIndex][tripIndex][i]][droneTripList[droneIndex][tripIndex][i+1]] + _takeoff_time + _landing_time;
            cusCompleteTime[droneTripList[droneIndex][tripIndex][i]] = newDroneTime;
        }
        newDroneTime += input.droneTimes[droneTripList[droneIndex][tripIndex].back()][0] + _takeoff_time + _landing_time;
        score.droneDemand[droneIndex][tripIndex] = newDemand;
        score.droneTime[droneIndex][tripIndex] = newDroneTime;

        //update waiting time
        for(int k : droneTripList[droneIndex][tripIndex]){
            newWaitingTime += std::max(0., newDroneTime - cusCompleteTime[k] - config2.sampleLimitationWaitingTime);
        }
        score.droneWaitingTime[droneIndex][tripIndex] = newWaitingTime;
        //update energy
        while (score.droneEnergy[droneIndex][tripIndex].size() != droneTripList[droneIndex][tripIndex].size()){
            if (score.droneEnergy[droneIndex][tripIndex].size() < droneTripList[droneIndex][tripIndex].size()){
                score.droneEnergy[droneIndex][tripIndex].emplace_back();
            }
            if (score.droneEnergy[droneIndex][tripIndex].size() > droneTripList[droneIndex][tripIndex].size()){
                score.droneEnergy[droneIndex][tripIndex].pop_back();
            }
        }
        double totalDemand = 0;
        int previousCus = 0;
        double totalEnergy = 0;
        for(int i = 0; i < index; i++){
            totalDemand += input.demand[droneTripList[droneIndex][tripIndex][i]];
            totalEnergy += score.droneEnergy[droneIndex][tripIndex][i];
        }
        if(index > 0){
                previousCus = droneTripList[droneIndex][tripIndex][index - 1];
            }
        for(int i = index; i < droneTripList[droneIndex][tripIndex].size(); i++){
            score.droneEnergy[droneIndex][tripIndex][i] = countEnergy(previousCus, droneTripList[droneIndex][tripIndex][index], totalDemand, input);
            totalEnergy += score.droneEnergy[droneIndex][tripIndex][i];
            totalDemand += input.demand[droneTripList[droneIndex][tripIndex][index]];
            previousCus = droneTripList[droneIndex][tripIndex][index];
        }
        totalEnergy += countEnergy(previousCus, 0, totalDemand, input);
        score.droneOverEnergy[droneIndex][tripIndex] = std::max(0., totalEnergy - config2.droneBatteryPower);
    }
}

void Score::updateTruckScore(std::vector<std::vector<std::vector<int>>> techTripList, Score &score, Input &input,
                                                                                int techIndex, int techTripIndex, int index){
    // update demand of truck
    double newDemand = 0;
    for (int i = 0; i < techTripList[techIndex][techTripIndex].size(); i++){
        newDemand += input.demand[techTripList[techIndex][techTripIndex][i]];
    }
    score.truckDemand[techIndex][techTripIndex] = newDemand;
    //update time
    for(int i = 0; i < techTripList[techIndex].size(); i++){
        while (score.truckTime[techIndex][i].size() < techTripList[techIndex][i].size()){
            score.truckTime[techIndex][i].emplace_back();
        }
        while (score.truckTime[techIndex][i].size() > techTripList[techIndex][i].size()){
            score.truckTime[techIndex][i].pop_back();
        }
    }

    double startTime = 0;
    if (techTripIndex == 0){
        if (index > 0){
            startTime = score.truckTime[techIndex][techTripIndex][index - 1];
        }
    }else if (techTripIndex > 0){
        if (index == 0){
            startTime = score.truckCompleteTime[techIndex][techTripIndex - 1];
        }else if (index > 0){
            startTime = score.truckTime[techIndex][techTripIndex][index - 1];
        }    
    }
    int previousCus = 0;
    if (index > 0){
        previousCus = techTripList[techIndex][techTripIndex][index - 1];
    }

    if (techTripList[techIndex][techTripIndex].empty()){
        score.truckCompleteTime[techIndex][techTripIndex] = startTime;
    }else{
        for (int i = index; i < techTripList[techIndex][techTripIndex].size(); i++){
            score.truckTime[techIndex][techTripIndex][i] = startTime + countTimeTruck1(startTime, input.distances[previousCus][techTripList[techIndex][techTripIndex][i]], input)
                        + input.serviceTimeByTruck[techTripList[techIndex][techTripIndex][i]];
            startTime = score.truckTime[techIndex][techTripIndex][i];
            previousCus = techTripList[techIndex][techTripIndex][i];
        }
        score.truckCompleteTime[techIndex][techTripIndex] = startTime + countTimeTruck1(startTime, input.distances[techTripList[techIndex][techTripIndex].back()][0], input);
        startTime = score.truckCompleteTime[techIndex][techTripIndex];
    }

    for( int i = techTripIndex + 1; i < techTripList[techIndex].size(); i++){
        previousCus = 0;
        if (techTripList[techIndex][i].empty()){
            score.truckCompleteTime[techIndex][i] = startTime;
            continue;
        }
        for( int j = 0; j < techTripList[techIndex][i].size(); j++){
            score.truckTime[techIndex][i][j] = startTime + countTimeTruck1(startTime, input.distances[previousCus][techTripList[techIndex][i][j]], input)
                    + input.serviceTimeByTruck[techTripList[techIndex][i][j]];
            startTime = score.truckTime[techIndex][i][j];
            previousCus = techTripList[techIndex][i][j];
        }
        score.truckCompleteTime[techIndex][i] = startTime + countTimeTruck1(startTime, input.distances[previousCus][0], input);
        startTime = score.truckCompleteTime[techIndex][i];
    }
}

