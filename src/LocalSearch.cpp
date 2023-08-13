//
// Created by MacBook Pro on 30/09/2022.
//
#include <chrono>

#include "LocalSearch.h"

using namespace std::chrono;

LocalSearch::LocalSearch(Config &conf, Input &inp) {
    this->config = conf;
    this->input = inp;
    this->alpha1 = conf.tabuAlpha1;
    this->alpha2 = conf.tabuAlpha2;
    Solution *init = Solution::initSolution(this->config, this->input, InitType::MIX, alpha1, alpha2);
    if (init == nullptr) {
        return;
    }
    initSolution = *init;
}

void LocalSearch::run(json &log) {
    currentSolution = initSolution;
    double currentScore = initSolution.getScore()[0][0][0];
    bestSolution = initSolution;
    double bestScore = currentScore;

    Solution *s;

    auto start = high_resolution_clock::now();

    for (int it = 0; it < config.tabuMaxIter; it++) {
        std::cout << it << std::endl;
        while (true) {
            bool isImproved = false;
            s = currentSolution.relocate({}, bestScore);
            if (s != nullptr) {
                double sScore = s->getScore()[0][0][0];
                if (sScore < bestScore) {
                    isImproved = true;
                    bestSolution = *s;
                    currentSolution = *s;

                    bestScore = sScore;
                    currentScore = sScore;
                }
            }

            s = currentSolution.exchange({}, bestScore);
            if (s != nullptr) {
                double sScore = s->getScore()[0][0][0];
                if (sScore < bestScore) {
                    isImproved = true;
                    bestSolution = *s;
                    currentSolution = *s;

                    bestScore = sScore;
                    currentScore = sScore;
                }
            }

            if (!isImproved) {
                break;
            }
        }
        currentSolution.perturbation();
    }
    auto stop = high_resolution_clock::now();
    json jDroneBest(bestSolution.droneTripList);
    json jTechBest(bestSolution.techTripList);
    log["ls_time"] = duration_cast<milliseconds>(stop - start).count();
    log["ls_best_score"] = bestScore;
    log["ls_best"] = jDroneBest.dump() + " || " + jTechBest.dump();
}
