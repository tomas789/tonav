//
// Created by Tomas Krejci on 10/14/17.
//

#ifndef TONAV_RUN_LOOP_H
#define TONAV_RUN_LOOP_H

#include <cmath>
#include <queue>

#include "run_loop_callback.h"

class VioSimulation;

class RunLoop {
public:
    double getTime() const;
    void setSimulationLength(double time);
    void run();
    void stop();
    
    void registerSimulationForUpdates(VioSimulation *simulation);
    void registerCallback(double time, RunLoopCallback *callback);
    
    bool isRunning() const;
    
private:
    struct Item {
        double time;
        RunLoopCallback *callback;
    
        bool operator>(const Item& other) const;
    };
    
    bool should_stop_ = false;
    double simulation_length_ = NAN;
    VioSimulation *simulation_ = nullptr;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue_;
};

#endif //TONAV_RUN_LOOP_H
