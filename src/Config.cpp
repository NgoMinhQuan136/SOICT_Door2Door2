//
// Created by MacBook Pro on 02/06/2022.
//

#include "Config.h"

Config::Config() {
    droneVelocity = 31.2928;
    droneWeight = 1.5;
    droneTakeoffSpeed = 15.6464;
    droneCruiseSpeed = 31.2928;
    droneLandingSpeed = 7.8232;
    cruiseAlt = 50;
    droneCapacity = 2.27;
    droneBatteryPower = 457503;

    k1 = 0.8554;
    k2 = 0.3051;
    c1 = 2.8037;
    c2 = 0.3177;
    c4 = 0.0296;
    c5 = 0.0279;

    techVelocity = 15.557;
    numDrone = 3;
    numTech = 3;
    minTabuDuration = 5;
    maxTabuDuration = 10;

    droneLimitationFightTime = 120000;
    sampleLimitationWaitingTime = 60000;

    tabuMaxIter = 200;
    tabuNotImproveIter = 200;
    tabuAlpha1 = 1;
    tabuAlpha2 = 1;
    tabuBeta = 0.5;
    tabuEpsilon = 1e-3;
    tabuNumRunPerDataSet = 1;
    maxEjectionLevel = 2;
    isCycle = false;
    dataPath = "data/";
    dataName = "6.5.2.txt";
    ws = "D:/Study/Lab/Soict/";
    resultFolder = ws + "result/";
    
}
