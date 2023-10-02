//
// Created by MacBook Pro on 30/09/2022.
//

#ifndef DOOR2DOOR2_LOCALSEARCH_H
#define DOOR2DOOR2_LOCALSEARCH_H

#include "Config.h"
#include "Input.h"
#include "iostream"
#include "Solution.h"
#include "nlohmann/json.hpp"
#include "Score.h"

using json = nlohmann::json;

class LocalSearch {
    Config config;
    Input input;
    Score score;

    Solution currentSolution;

    double alpha1{};
    double alpha2{};

public:
    LocalSearch(Config &conf, Input &inp);

    Solution initSolution;

    void run(json &log);

    Solution bestSolution;
};


#endif //DOOR2DOOR2_LOCALSEARCH_H
