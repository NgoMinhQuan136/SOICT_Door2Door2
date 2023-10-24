//
// Created by MacBook Pro on 02/06/2022.
//

#ifndef DOOR2DOOR2_TABUSEARCH_H
#define DOOR2DOOR2_TABUSEARCH_H


#include "Config.h"
#include "Input.h"
#include "iostream"
#include "Solution.h"
#include "nlohmann/json.hpp"
#include "Score.h"

using json = nlohmann::json;

class TabuSearch {
public:
    Config config;
    Input input;

    Solution initSolution; //lời giải khởi tạo ban đầu
    Solution currentSolution; // lời giải đang xét 
    Solution bestFeasibleSolution; // lời giải feasible, tốt nhất từ đầu đến giờ

    Score score;
    Score score1;

    int tabuDuration{};
    double alpha1{};
    double alpha2{};
    double alpha3{};

    TabuSearch();

    TabuSearch(Config &conf, Input &inp);

    void run(json &log);

    void runPostOptimization(json &log);

    static void runEjection(Solution &solution);

    static void runInterRoute(Solution &solution);

    static void runIntraRoute(Solution &solution);

    void updatePenalty(double dz, double cz, double ez);
};


#endif //DOOR2DOOR2_TABUSEARCH_H
