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

void countMove(std::vector<int> &moveCount, int move){
    for( int i = 1; i <= 5; i++){
        if (move == i ){
            moveCount[i] ++;
        }
    }
}

bool checkMove(std::vector<int> moveCount, int move, int cycle){
    if ( moveCount[move] > cycle){
        return true;
    }else{
        return false;
    }
    
}

void TabuSearch::run(json &log) {
    std::map<NeighborhoodType, std::vector<std::string>> tabuLists;
    std::vector<int> moveCount(6, 0);
    tabuLists[MOVE_10] = std::vector<std::string>();
    tabuLists[MOVE_11] = std::vector<std::string>();
    tabuLists[MOVE_20] = std::vector<std::string>();
    tabuLists[MOVE_21] = std::vector<std::string>();
    tabuLists[TWO_OPT] = std::vector<std::string>();

    int notImproveIter = 0;
    int findBest = 0;
    bool isImproved;
    double testScore = 0;
    currentSolution = initSolution;
    std::vector<std::vector<std::vector<double>>> point = initSolution.getScore(score);
    double currentScore = point[0][0][0];
    std::cout << "CurScore :" << currentScore << "\n";
    std::cout << "NewScore :" << currentSolution.getNewScore(score) << "\n";

    bestFeasibleSolution = currentSolution;
    double bestFeasibleScore = currentScore; // trong trường hợp init không phải feasible solution ?
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
            double cycle = it / 5;
            do{
                actOrd = Random::get(1, 5);
            } while (checkMove(moveCount, actOrd, cycle));
            
            
        }
        json itLog;
        itLog["act"] = actOrd;
        countMove(moveCount, actOrd);
        switch (actOrd) {
            case MOVE_10: {
                neighborhoodType = MOVE_10;
                isImproved = false;
                s = currentSolution.relocate(bestFeasibleSolution, isImproved, score, tabuLists[MOVE_10], bestFeasibleScore);
                break;
            }
            case MOVE_11: {
                neighborhoodType = MOVE_11;
                isImproved = false;
                s = currentSolution.exchange(bestFeasibleSolution, isImproved, score, tabuLists[MOVE_11], bestFeasibleScore);
                break;
            }
            case MOVE_20: {
                neighborhoodType = MOVE_20;
                isImproved = false;
                s = currentSolution.orOpt(bestFeasibleSolution, isImproved, score, tabuLists[MOVE_20], bestFeasibleScore);
                break;
            }
            case MOVE_21: {
                neighborhoodType = MOVE_21;
                isImproved = false;
                s = currentSolution.crossExchange(bestFeasibleSolution, isImproved, score, tabuLists[MOVE_21], bestFeasibleScore);
                break;
            }
            case TWO_OPT: {
                neighborhoodType = TWO_OPT;
                isImproved = false;
                s = currentSolution.twoOpt(bestFeasibleSolution, isImproved, score, tabuLists[TWO_OPT], bestFeasibleScore);
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
            // json jDroneOld(currentSolution.droneTripList);
            // json jTechOld(currentSolution.techTripList);
            // currentScore = currentSolution.getScore()[0][0][0];
            // itLog["old"] = std::to_string(currentScore) + " == " + jDroneOld.dump() + " || " + jTechOld.dump();
            currentSolution = *s;
            std::vector<std::vector<std::vector<double>>> pointSolution = currentSolution.getScore(score1);
            currentScore = pointSolution[0][0][0];
            // testScore = currentSolution.getScore(score1)[0][0][0];
            json jDrone(currentSolution.droneTripList);
            json jTech(currentSolution.techTripList);
            json jDroneDemand(pointSolution[1]);
            json jTruckDemand(pointSolution[2]);
            json jDroneWaitingTime(pointSolution[3]);
            json jTruckWaitingTime(pointSolution[4]);
            json jOverEnergy(pointSolution[5]);

            
            // std::cout << " Score : " << pointSolution[0][0][0] << "\n";
            // std::cout << " New Score : " << currentScore <<"\n";

            // for(int i = 0; i < score.truckCompleteTime.size(); i++){
            //     std::cout << " Truck Time : " << i << " : " <<"\n";
            //     for(int j = 0; j < score.truckCompleteTime[i].size(); j++){
            //         std::cout << score.truckCompleteTime[i][j] <<" - ";
            //         for(int k = 0; k < score.truckTime[i][j].size(); k ++){
            //             std::cout << score.truckTime[i][j][k] << " | ";
            //         }
            //         std::cout <<"\n";  
            //     }
            //     std::cout <<"\n";  
            // }

            // for (int i = 0; i < pointSolution[1].size(); i++){
            //     std::cout << " Truck Time 1 : " << i << " : ";
            //     for(int j = 0; j < pointSolution[1][i].size(); j++){
            //         std::cout << pointSolution[1][i][j] <<" - ";
            //     }
            //     std::cout <<"\n";        
            // }

            // for(int i = 0; i < score1.truckCompleteTime.size(); i++){
            //     std::cout << " Truck Time A : " << i << " : " <<"\n";
            //     for(int j = 0; j < score1.truckCompleteTime[i].size(); j++){
            //         std::cout << score1.truckCompleteTime[i][j] <<" - ";
            //         for(int k = 0; k < score1.truckTime[i][j].size(); k ++){
            //             std::cout << score1.truckTime[i][j][k] << " | ";
            //         }
            //         std::cout <<"\n";  
            //     }
            //     std::cout <<"\n";  
            // }
            
            itLog["current"] = std::to_string(currentScore) + " == " + jDrone.dump() + " || " + jTech.dump();
            // itLog["current_test"] = std::to_string(testScore);
            // itLog["cur_Demand"] = jDroneDemand.dump() + " || " + jTruckDemand.dump();
            // itLog["cur_WaitingTime"] = jDroneWaitingTime.dump() + " || " + jTruckWaitingTime.dump();
            // itLog["cur_OverEnergy"] = jOverEnergy.dump();
            itLog["cur_timeTech"] = std::to_string(pointSolution[0][0][6]);
            itLog["cur_timeDrone"] = std::to_string(pointSolution[0][0][5]);
            itLog["cur_penalty"] = std::to_string(pointSolution[0][0][2]) + " || " + std::to_string(pointSolution[0][0][3]) + " || " 
                + std::to_string(pointSolution[0][0][4]);
            if (isImproved) {
                // bestFeasibleScore = bestFeasibleSolution.getNewScore(score);
                notImproveIter = 0;
                alpha1 = config.tabuAlpha1;
                alpha2 = config.tabuAlpha2;
                alpha3 = config.tabuAlpha3;
                currentSolution.alpha1 = alpha1;
                currentSolution.alpha2 = alpha2;
                currentSolution.alpha3 = alpha3;

                findBest = it;
            } else {
                notImproveIter++;
                updatePenalty(currentSolution.dz, currentSolution.cz, currentSolution.ez);
                // std::cout << alpha1 << "| " << alpha2 << " |" << alpha3 << "|\n";
                currentSolution.alpha1 = alpha1;
                currentSolution.alpha2 = alpha2;
                currentSolution.alpha3 = alpha3;
                if (notImproveIter > config.tabuNotImproveIter) {
                    break;
                }
            }
            json jDroneBest(bestFeasibleSolution.droneTripList);
            json jTechBest(bestFeasibleSolution.techTripList);
            json jDroneBestFeasible(bestFeasibleSolution.droneTripList);
            json jTechBestFeasible(bestFeasibleSolution.techTripList);
            // itLog["best"] = std::to_string(bestFeasibleScore) + " == " + jDroneBest.dump() + " || " + jTechBest.dump();
            // itLog["best_feasible"] = std::to_string(findBest) + " : " + std::to_string(bestFeasibleScore) + " == " + jDroneBestFeasible.dump() + " || " + jTechBestFeasible.dump();
        }

        log[std::to_string(it)] = itLog;
    }
    auto stop = high_resolution_clock::now();
    json jDroneBest(bestFeasibleSolution.droneTripList);
    json jTechBest(bestFeasibleSolution.techTripList);
    json jDroneBestFeasible(bestFeasibleSolution.droneTripList);
    json jTechBestFeasible(bestFeasibleSolution.techTripList);
    log["tabu_time"] = duration_cast<milliseconds>(stop - start).count();
    log["best_tabu"] =std::to_string(findBest) + " : " + std::to_string(bestFeasibleScore) + " == " + jDroneBestFeasible.dump() + " || " + jTechBestFeasible.dump();
}

TabuSearch::TabuSearch(Config &conf, Input &inp) {
    this->config = conf;
    this->input = inp;
    this->tabuDuration = Random::get(config.minTabuDuration, config.maxTabuDuration);
    this->alpha1 = conf.tabuAlpha1;
    this->alpha2 = conf.tabuAlpha2;
    this->alpha3 = conf.tabuAlpha3;
    Solution *init = Solution::initSolution(this->config, this->input, InitType::MIX, alpha1, alpha2, alpha3);
    if (init == nullptr) {
        return;
    }
    initSolution = *init;
}

void TabuSearch::updatePenalty(double dz, double cz, double ez) { 
    if (dz > 0) {
        alpha1 *= 1 + config.tabuBeta;
        if(alpha1 > 10000000){
            alpha1 = 1;
        }
    } else {
        alpha1 /= 1 + config.tabuBeta;
        if(alpha1 < 0.000001){
            alpha1 = 1;
        }
    }

    if (cz > 0) {
        alpha2 *= 1 + config.tabuBeta;
        if(alpha2 > 10000000){
            alpha2 = 1;
        }
    } else {
        alpha2 /= 1 + config.tabuBeta;
        if (alpha2 < 0.000001){
            alpha2 = 1;
        }
    }

    if (ez > 0) {
        alpha3 *= 1 + config.tabuBeta;
        if(alpha3 > 10000000){
            alpha3 = 1;
        }
    } else {
        alpha3 /= 1 + config.tabuBeta;
        if(alpha3 < 0.000001){
            alpha3 = 1;
        }
    }
}

// void TabuSearch::runPostOptimization(json &log) {
//     auto start = high_resolution_clock::now();

//     runEjection(bestFeasibleSolution);
//     // std::cout << "Ejection: "  << std::endl;
//     json jDroneBestEjection(bestFeasibleSolution.droneTripList);
//     json jTechBestEjection(bestFeasibleSolution.techTripList);
//     log["best_ejection"] = std::to_string(bestFeasibleSolution.getScore(score)[0][0][0]) + " == "
//                            + jDroneBestEjection.dump() + " || "
//                            + jTechBestEjection.dump();

// //    std::cout << "Ejection: " << log["best_ejection"].dump(4) << std::endl;

//     runInterRoute(bestFeasibleSolution);
//     json jDroneBestInter(bestFeasibleSolution.droneTripList);
//     json jTechBestInter(bestFeasibleSolution.techTripList);
//     log["best_inter"] = std::to_string(bestFeasibleSolution.getScore(score)[0][0][0]) + " == "
//                         + jDroneBestInter.dump() + " || "
//                         + jTechBestInter.dump();

// //    std::cout << "Inter: " << log["best_inter"].dump(4) << std::endl;
//     runIntraRoute(bestFeasibleSolution);
//     json jDroneBestIntra(bestFeasibleSolution.droneTripList);
//     json jTechBestIntra(bestFeasibleSolution.techTripList);
//     log["best_intra"] = std::to_string(bestFeasibleSolution.getScore(score)[0][0][0]) + " == "
//                         + jDroneBestIntra.dump() + " || "
//                         + jTechBestIntra.dump();
// //    std::cout << "Intra: " << log["best_intra"].dump(4) << std::endl;
//     auto stop = high_resolution_clock::now();
//     log["post_optimization_time"] = duration_cast<milliseconds>(stop - start).count();
// }

// void TabuSearch::runInterRoute(Solution &solution) {
//     auto rng = std::default_random_engine(std::chrono::system_clock::now()
//                                                   .time_since_epoch()
//                                                   .count());
//     std::vector<InterRouteType> order{INTER_RELOCATE, INTER_CROSS_EXCHANGE, INTER_EXCHANGE, INTER_OR_OPT,
//                                       INTER_TWO_OPT};
    

//     double score = solution.getScore()[0][0][0];
//     double newScore;

//     Solution *s;
//     while (true) {
//         std::shuffle(order.begin(), order.end(), rng);
//         bool hasImprove = false;

//         for (InterRouteType type: order) {
//             switch (type) {
//                 case INTER_RELOCATE: {
//                     // s = solution.relocate({}, score, INTER);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//                 case INTER_EXCHANGE: {
//                     // s = solution.exchange({}, score, INTER);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//                 case INTER_OR_OPT: {
//                     // s = solution.orOpt({}, score, INTER);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//                 case INTER_TWO_OPT: {
//                     // s = solution.twoOpt({}, score, INTER);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//                 case INTER_CROSS_EXCHANGE: {
//                     // s = solution.crossExchange({}, score, INTER);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//             }
//         }

//         if (!hasImprove) {
//             break;
//         }
//     }
//     // std::cout <<"Done Inter Route" << "\n";
// }

// void TabuSearch::runIntraRoute(Solution &solution) {
//     auto rng = std::default_random_engine(std::chrono::system_clock::now()
//                                                   .time_since_epoch()
//                                                   .count());
//     std::vector<IntraRouteType> order{INTRA_RELOCATE, INTRA_EXCHANGE, INTRA_OR_OPT, INTRA_TWO_OPT};


//     double score = solution.getScore()[0][0][0];
//     double newScore;

//     Solution *s;
//     while (true) {
//         std::shuffle(order.begin(), order.end(), rng);
//         bool hasImprove = false;

//         for (IntraRouteType type: order) {
//             switch (type) {
//                 case INTRA_RELOCATE: {
//                     // s = solution.relocate({}, score, INTRA);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//                 case INTRA_EXCHANGE: {
//                     // s = solution.exchange({}, score, INTRA);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//                 case INTRA_TWO_OPT: {
//                     // s = solution.twoOpt({}, score, INTRA);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//                 case INTRA_OR_OPT: {
//                     // s = solution.orOpt({}, score, INTRA);
//                     if (s != nullptr) {
//                         newScore = s->getScore()[0][0][0];
//                         if (newScore < score) {
//                             solution = *s;
//                             score = newScore;
//                             hasImprove = true;
//                         }
//                     }
//                     break;
//                 }
//             }
//         }

//         if (!hasImprove) {
//             break;
//         }
//     }
// }

void TabuSearch::runEjection(Solution &solution) {
    solution = solution.ejection();
}

TabuSearch::TabuSearch() = default;