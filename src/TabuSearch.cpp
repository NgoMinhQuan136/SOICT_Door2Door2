//
// Created by MacBook Pro on 02/06/2022.
//

#include "TabuSearch.h"
#include "Solution.h"
#include "Random.h"
#include <chrono>
#include "nlohmann/json.hpp"

using namespace std::chrono;

using Random = effolkronium::random_static;
using json = nlohmann::json;

void TabuSearch::run(json &log) {
    std::map<NeighborhoodType, std::vector<std::string>> tabuLists;
    tabuLists[MOVE_10] = std::vector<std::string>();
    tabuLists[MOVE_11] = std::vector<std::string>();
    tabuLists[MOVE_20] = std::vector<std::string>();
    tabuLists[MOVE_21] = std::vector<std::string>();
    tabuLists[TWO_OPT] = std::vector<std::string>();

    int notImproveIter = 0;
    currentSolution = initSolution;
    double currentScore = initSolution.getScore();
    bestSolution = initSolution;
    double bestScore = currentScore;
    NeighborhoodType neighborhoodType;
    int actOrd;
    Solution *s;

    auto start = high_resolution_clock::now();
    int actOrderCycle = -1;

    for (int it = 0; it < config.tabuMaxIter; it++) {
        actOrderCycle = (actOrderCycle + 1) % 5;
        if (config.isCycle) {
            actOrd = actOrderCycle + 1;
        } else {
            actOrd = Random::get(1, 5);
        }
//        actOrd = 4;
        json itLog;
        itLog["act"] = actOrd;
//       std::cout << "act: " << actOrd << std::endl;
//        auto start = high_resolution_clock::now();
        switch (actOrd) {
            case MOVE_10: {
                neighborhoodType = MOVE_10;
                s = currentSolution.relocate(tabuLists[MOVE_10], bestScore);

                break;
            }
            case MOVE_11: {
                neighborhoodType = MOVE_11;
                s = currentSolution.exchange(tabuLists[MOVE_11], bestScore);

                break;
            }
            case MOVE_20: {
                neighborhoodType = MOVE_20;
                s = currentSolution.orOpt(tabuLists[MOVE_20], bestScore);

                break;
            }
            case MOVE_21: {
                neighborhoodType = MOVE_21;
                s = currentSolution.crossExchange(tabuLists[MOVE_21], bestScore);

                break;
            }
            case TWO_OPT: {
                neighborhoodType = TWO_OPT;
                s = currentSolution.twoOpt(tabuLists[TWO_OPT], bestScore);

                break;
            }
            default: {
                s = nullptr;
                break;
            }
        }

        if (s != nullptr) {
            itLog["ext"] = s->ext["state"];
            tabuLists[neighborhoodType].push_back(s->ext["state"]);
            while (tabuLists[neighborhoodType].size() > tabuDuration) {
                tabuLists[neighborhoodType].erase(tabuLists[neighborhoodType].begin());
            }
            json jDroneOld(currentSolution.droneTripList);
            json jTechOld(currentSolution.techTripList);
            currentScore = currentSolution.getScore();
            itLog["old"] = std::to_string(currentScore) + " == " + jDroneOld.dump() + " || " + jTechOld.dump();
            currentSolution = *s;
            currentScore = currentSolution.getScore();
            json jDrone(currentSolution.droneTripList);
            json jTech(currentSolution.techTripList);
            itLog["current"] = std::to_string(currentScore) + " == " + jDrone.dump() + " || " + jTech.dump();
            if (currentScore < bestScore) {
                bestSolution = currentSolution;
                bestScore = currentScore;
                notImproveIter = 0;
                updatePenalty(bestSolution.dz, bestSolution.dz);
                bestSolution.alpha1 = alpha1;
                bestSolution.alpha2 = alpha2;
                currentSolution.alpha1 = alpha1;
                currentSolution.alpha2 = alpha2;
            } else {
                notImproveIter++;
                if (notImproveIter > config.tabuNotImproveIter) {
                    break;
                }
            }
            json jDroneBest(bestSolution.droneTripList);
            json jTechBest(bestSolution.techTripList);
            itLog["best"] = std::to_string(bestScore) + " == " + jDroneBest.dump() + " || " + jTechBest.dump();
        }

//        auto stop = high_resolution_clock::now();
//        std::cout << it << ": "
//                  << bestScore << "-" << neighborhoodType << " time: "
//                  << duration_cast<seconds>(stop - start).count() << "s" << std::endl;

        log[std::to_string(it)] = itLog;
//        std::cout << it << ": " << itLog.dump(4) << std::endl;
    }
    auto stop = high_resolution_clock::now();
    json jDroneBest(bestSolution.droneTripList);
    json jTechBest(bestSolution.techTripList);
    log["tabu_time"] = duration_cast<milliseconds>(stop - start).count();
    log["best_tabu"] = std::to_string(bestScore) + " == " + jDroneBest.dump() + " || " + jTechBest.dump();
}

TabuSearch::TabuSearch(Config &conf, Input &inp) {
    this->config = conf;
    this->input = inp;
    this->tabuDuration = Random::get(config.minTabuDuration, config.maxTabuDuration);
    this->alpha1 = conf.tabuAlpha1;
    this->alpha2 = conf.tabuAlpha2;
    Solution *init = Solution::initSolution(this->config, this->input, InitType::MIX, alpha1, alpha2);
    if (init == nullptr) {
        return;
    }
    initSolution = *init;
}

void TabuSearch::updatePenalty(double dz, double cz) {
    if (dz > 0) {
        alpha1 *= 1 + config.tabuBeta;
    } else {
        alpha1 /= 1 + config.tabuBeta;
    }

    if (cz > 0) {
        alpha2 *= 1 + config.tabuBeta;
    } else {
        alpha2 /= 1 + config.tabuBeta;
    }
}

void TabuSearch::runPostOptimization(json &log) {
    auto start = high_resolution_clock::now();

    runEjection(bestSolution);
    json jDroneBestEjection(bestSolution.droneTripList);
    json jTechBestEjection(bestSolution.techTripList);
    log["best_ejection"] = std::to_string(bestSolution.getScore()) + " == "
                           + jDroneBestEjection.dump() + " || "
                           + jTechBestEjection.dump();

//    std::cout << "Ejection: " << log["best_ejection"].dump(4) << std::endl;

    runInterRoute(bestSolution);
    json jDroneBestInter(bestSolution.droneTripList);
    json jTechBestInter(bestSolution.techTripList);
    log["best_inter"] = std::to_string(bestSolution.getScore()) + " == "
                        + jDroneBestInter.dump() + " || "
                        + jTechBestInter.dump();

//    std::cout << "Inter: " << log["best_inter"].dump(4) << std::endl;
    runIntraRoute(bestSolution);
    json jDroneBestIntra(bestSolution.droneTripList);
    json jTechBestIntra(bestSolution.techTripList);
    log["best_intra"] = std::to_string(bestSolution.getScore()) + " == "
                        + jDroneBestIntra.dump() + " || "
                        + jTechBestIntra.dump();
//    std::cout << "Intra: " << log["best_intra"].dump(4) << std::endl;
    auto stop = high_resolution_clock::now();
    log["post_optimization_time"] = duration_cast<milliseconds>(stop - start).count();
}

void TabuSearch::runInterRoute(Solution &solution) {
    auto rng = std::default_random_engine(std::chrono::system_clock::now()
                                                  .time_since_epoch()
                                                  .count());
    // std::vector<InterRouteType> order{INTER_RELOCATE, INTER_CROSS_EXCHANGE, INTER_EXCHANGE, INTER_OR_OPT,
    //                                   INTER_TWO_OPT};
    
    std::vector<InterRouteType> order{INTER_RELOCATE, INTER_EXCHANGE, INTER_OR_OPT, INTER_CROSS_EXCHANGE};

    double score = solution.getScore();
    double newScore;

    Solution *s;
    while (true) {
        std::shuffle(order.begin(), order.end(), rng);
        bool hasImprove = false;

        for (InterRouteType type: order) {
            switch (type) {
                case INTER_RELOCATE: {
                    s = solution.relocate({}, score, INTER);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
                case INTER_EXCHANGE: {
                    s = solution.exchange({}, score, INTER);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
                case INTER_OR_OPT: {
                    s = solution.orOpt({}, score, INTER);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
                case INTER_TWO_OPT: {
                    s = solution.twoOpt({}, score, INTER);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
                case INTER_CROSS_EXCHANGE: {
                    s = solution.crossExchange({}, score, INTER);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
            }
        }

        if (!hasImprove) {
            break;
        }
    }
    std::cout <<"Done Inter Route" << "\n";
}

void TabuSearch::runIntraRoute(Solution &solution) {
    auto rng = std::default_random_engine(std::chrono::system_clock::now()
                                                  .time_since_epoch()
                                                  .count());
    // std::vector<IntraRouteType> order{INTRA_RELOCATE, INTRA_EXCHANGE, INTRA_OR_OPT, INTRA_TWO_OPT};
    std::vector<IntraRouteType> order{INTRA_RELOCATE, INTRA_EXCHANGE, INTRA_OR_OPT};

    double score = solution.getScore();
    double newScore;

    Solution *s;
    while (true) {
        std::shuffle(order.begin(), order.end(), rng);
        bool hasImprove = false;

        for (IntraRouteType type: order) {
            switch (type) {
                case INTRA_RELOCATE: {
                    s = solution.relocate({}, score, INTRA);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
                case INTRA_EXCHANGE: {
                    s = solution.exchange({}, score, INTRA);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
                case INTRA_TWO_OPT: {
                    s = solution.twoOpt({}, score, INTRA);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
                case INTRA_OR_OPT: {
                    s = solution.orOpt({}, score, INTRA);
                    if (s != nullptr) {
                        newScore = s->getScore();
                        if (newScore < score) {
                            solution = *s;
                            score = newScore;
                            hasImprove = true;
                        }
                    }
                    break;
                }
            }
        }

        if (!hasImprove) {
            break;
        }
    }
}

void TabuSearch::runEjection(Solution &solution) {
    solution = solution.ejection();
}

TabuSearch::TabuSearch() = default;