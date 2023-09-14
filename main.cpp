#include "src/Config.h"
#include "src/Input.h"
#include "nlohmann/json.hpp"
#include "src/TabuSearch.h"
#include "src/Utils.h"
#include "src/LocalSearch.h"
#include <fstream>

using json = nlohmann::json;

int main(int argc, char **argv)
{
    Config config;
    std::string configFilePath = "D:\\Study\\Lab\\Soictv1\\config.json";
    if (argc == 2)
    {
        configFilePath = argv[1];
    }
    std::ifstream configFile(configFilePath);
    json jConfig = json::parse(configFile);
    //        configFile >> jConfig;
    config.droneVelocity = jConfig["droneVelocity"].get<double>();
    config.techVelocity = jConfig["techVelocity"].get<double>();
    config.numDrone = jConfig["numDrone"].get<int>();
    config.numTech = jConfig["numTech"].get<int>();
    config.minTabuDuration = jConfig["minTabuDuration"].get<int>();
    config.maxTabuDuration = jConfig["maxTabuDuration"].get<int>();
    config.droneLimitationFightTime = jConfig["droneLimitationFightTime"].get<int>();
    config.sampleLimitationWaitingTime = jConfig["sampleLimitationWaitingTime"].get<int>();
    config.tabuMaxIter = jConfig["tabuMaxIter"].get<int>();
    config.tabuNotImproveIter = jConfig["tabuNotImproveIter"].get<int>();
    config.tabuAlpha1 = jConfig["tabuAlpha1"].get<double>();
    config.tabuAlpha2 = jConfig["tabuAlpha2"].get<double>();
    config.tabuAlpha3 = jConfig["tabuAlpha3"].get<double>();
    config.tabuBeta = jConfig["tabuBeta"].get<double>();
    config.tabuEpsilon = jConfig["tabuEpsilon"].get<double>();
    config.tabuNumRunPerDataSet = jConfig["tabuNumRunPerDataSet"].get<int>();
    config.maxEjectionLevel = jConfig["maxEjectionLevel"].get<int>();
    config.isCycle = jConfig["isCycle"].get<int>() == 1;
    config.type = jConfig["type"].get<std::string>();
    config.dataPath = jConfig["dataPath"].get<std::string>();
    config.dataName = jConfig["dataName"].get<std::string>();
    config.ws = jConfig["ws"].get<std::string>();
    config.resultFolder = jConfig["resultFolder"].get<std::string>();
    std::vector<std::string> paths = Utils::glob(config.ws + config.dataPath, config.dataName);

    json logAll;
    std::string truck_path = "D:\\Study\\Lab\\Soictv1\\data\\Truck_config.json";
    for (const std::string &path : paths)
    {
        std::cout << path << std::endl;
        Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFightTime, path);
        input.truck_Input(input, truck_path);
        json logDataSet;
        for (int run = 0; run < config.tabuNumRunPerDataSet; run++)
        {
            std::cout << "start run: " << run + 1 << std::endl;
            if (config.type == "lcs")
            {   
                LocalSearch localSearch(config, input);
                if (localSearch.initSolution.droneTripList.empty() && localSearch.initSolution.techTripList.empty())
                {
                    std::cout << "Infeasible!" << std::endl;
                    return 1;
                }
                json log;
                log["num_tech"] = config.numTech;
                log["num_drone"] = config.numDrone;
                log["data_set"] = input.dataSet;
                try
                {
                    std::cout << "=> run lcs: " << run + 1 << std::endl;
                    localSearch.run(log);
                    std::cout << "=> done lcs: " << run + 1 << std::endl;
                }
                catch (...)
                {
                    std::cout << "loi khi chay lcs!" << std::endl;
                }

                std::ofstream o(
                    config.resultFolder + "result_" + input.dataSet + "_" + std::to_string(run + 1) + ".json");
                o << std::setw(4) << log << std::endl;
                json logRun;
                logRun["score"] = localSearch.bestSolution.getScore();
                logDataSet[std::to_string(run)] = logRun;
                o.close();
            }
            else
            {   
                TabuSearch tabuSearch(config, input);
                if (tabuSearch.initSolution.droneTripList.empty() && tabuSearch.initSolution.techTripList.empty())
                {
                    std::cout << "Infeasible!" << std::endl;
                    return 1;
                }
                json log;
                log["num_tech"] = config.numTech;
                log["num_drone"] = config.numDrone;
                log["data_set"] = input.dataSet;
                try
                {
                    std::cout << "=> run tabu: " << run + 1 << std::endl;
                    tabuSearch.run(log);
                    std::cout << "=> done tabu: " << run + 1 << std::endl;
                }
                catch (...)
                {
                    std::cout << "loi khi chay tabu!" << std::endl;
                }
                // try
                // {
                //     std::cout << "=> run post optim: " << run + 1 << std::endl;
                //     tabuSearch.runPostOptimization(log);
                //     std::cout << "=> done post optim: " << run + 1 << std::endl;
                // }
                // catch (...)
                // {
                //     std::cout << "loi khi chay post optim!" << std::endl;
                // }

                std::ofstream o(
                    config.resultFolder + "result_" + input.dataSet + "_" + std::to_string(run + 1) + ".json");
                o << std::setw(4) << log << std::endl;
                json logRun;
                logRun["score"] = tabuSearch.bestFeasibleSolution.getScore();
                // logRun["time"] = (unsigned int)log["tabu_time"] + (unsigned int)log["post_optimization_time"];
                logDataSet[std::to_string(run)] = logRun;
                o.close();
            }
            std::cout << "=> done run: " << run + 1 << std::endl;
        }

        logAll[input.dataSet] = logDataSet;
    }

    std::ofstream o(config.resultFolder + "./result_all.json");
    o << std::setw(4) << logAll << std::endl;
    return 0;
}
