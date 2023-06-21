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
    int droneLimitationFightTime; //minutes
    int sampleLimitationWaitingTime; //minutes
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

    float k1 ;
    float k2 ;
    float c1 ;
    float c2 ;
    float c4 ;
    float c5 ;

    float W ; // drone weight; 
    float g ; // gravitational constant;
    float alpha ; // angle off attack;
    float landing_speed ;
    float takeoff_speed ;
    float drone_speed ; //(m/s)
    float h ;
    float drone_energy ;
    Config();
};


#endif //DOOR2DOOR2_CONFIG_H
