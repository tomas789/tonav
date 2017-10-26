//
// Created by Tomas Krejci on 10/14/17.
//

#ifndef TONAV_RUN_LOOP_H
#define TONAV_RUN_LOOP_H

#include <queue>

#include "run_loop_callback.h"

class VioSimulation;

class RunLoop {
public:
    float getTime() const;
    void run();
    
    void registerSimulationForUpdates(VioSimulation *simulation);
    void registerCallback(float time, RunLoopCallback *callback);
    
private:
    struct Item {
        float time;
        RunLoopCallback *callback;
    
        bool operator>(const Item& other) const;
    };
    
    VioSimulation *simulation_ = nullptr;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue_;
};

#endif //TONAV_RUN_LOOP_H
