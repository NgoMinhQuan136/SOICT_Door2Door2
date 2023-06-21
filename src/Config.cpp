//
// Created by MacBook Pro on 02/06/2022.
//

#include "Config.h"

Config::Config() {
    droneVelocity = 0.83;
    techVelocity = 0.58;
    numDrone = 3;
    numTech = 3;
    minTabuDuration = 5;
    maxTabuDuration = 10;

    droneLimitationFightTime = 120;
    sampleLimitationWaitingTime = 60;

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
    ws = "C:\\Users\\thuypt\\Documents\\ManhPP_Workspace\\thesis\\Door2Door2\\";
    resultFolder = ws + "result/";
    
}
