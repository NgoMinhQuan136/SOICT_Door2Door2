//
// Created by MacBook Pro on 02/06/2022.
//

#ifndef DOOR2DOOR2_CONFIG_H
#define DOOR2DOOR2_CONFIG_H


#include <string>

class Config {
public:
    double droneVelocity;
    double techVelocity;
    int droneLimitationFightTime; 
    int sampleLimitationWaitingTime; 
    int numTech;
    int numDrone;

    std::string type;
    std::string resultFolder;
    std::string dataPath;
    std::string dataName;
    std::string ws;

    int minTabuDuration;
    int maxTabuDuration;

    int tabuMaxIter;
    int tabuNotImproveIter;

    int tabuNumRunPerDataSet;
    double tabuAlpha1;
    double tabuAlpha2;
    double tabuBeta;
    double tabuEpsilon;

    int maxEjectionLevel;
    bool isCycle;

    double k1 ;
    double k2 ;
    double c1 ;
    double c2 ;
    double c4 ;
    double c5 ;

    double droneWeight ; // drone weight; 
    double g ; // gravitational constant;
    double alpha ; // angle off attack;
    double droneTakeoffSpeed ;
    double droneCruiseSpeed ;
    double droneLandingSpeed ; //(m/s)
    double cruiseAlt ;
    double droneCapacity ;
    double droneBatteryPower;
    Config();
};


#endif //DOOR2DOOR2_CONFIG_H
