
#ifndef DOOR2DOOR2_SCORE_H
#define DOOR2DOOR2_SCORE_H

#include <vector>
#include "Input.h"
#include "Config.h"

class Score {
public:
    
    //drone
    std::vector<std::vector<double>> droneTime;// Thời gian hoàn thành mỗi trip Drone
    std::vector<std::vector<double>> droneDemand;// Demand từng trip Drone
    std::vector<std::vector<double>> droneWaitingTime;// Tổng thời gian chờ vượt quá mỗi trip Drone (=0 nếu không vượt quá)
    std::vector<std::vector<std::vector<double>>> droneEnergy; // Năng lượng tiêu hao giữa 2 khách
    std::vector<std::vector<double>> droneOverEnergy;// Năng lượng vượt quá mỗi trip Drone (=0 nếu không vượt quá)

    //truck
    std::vector<std::vector<std::vector<double>>> truckTime; // Thời điểm hoàn thành mỗi khách của Truck
    std::vector<std::vector<double>> truckDemand; // Tổng Demand 1 trip Truck
    std::vector<std::vector<double>> truckCompleteTime; // Thời điểm hoàn thành 1 trip Truck

    double countEnergy(int cus1, int cus2, double demand, Input &input);
    int check_ratio1(double time, Input &input);
    double countTimeTruck1(double startTime, double distance, Input &input);
    void updateDroneScore(std::vector<std::vector<std::vector<int>>> droneTripList, Score &score, Input &input, int droneIndex, int tripIndex, int index);
    void updateTruckScore(std::vector<std::vector<std::vector<int>>> techTripList, Score &score, Input &input, int techIndex, int techTripIndex, int index);

};

#endif //DOOR2DOOR2_SCORE_H