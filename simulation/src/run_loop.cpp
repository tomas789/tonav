//
// Created by Tomas Krejci on 10/14/17.
//

#include "run_loop.h"

#include <limits>

#include "vio_simulation.h"

void RunLoop::run() {
    float last_simulation_update_time = std::numeric_limits<float>::min();
    while (!queue_.empty()) {
        Item item = queue_.top();
        queue_.pop();
        
        float time = item.time;
        RunLoopCallback *callback = item.callback;
        callback->runLoopCallback(time);
        
        std::cout << time << std::endl;
        std::cout << queue_.size() << std::endl;
        
        if (last_simulation_update_time != time) {
            if (simulation_ != nullptr) {
                simulation_->runLoopCallback(time);
            }
            last_simulation_update_time = time;
        }
    }
}

void RunLoop::registerSimulationForUpdates(VioSimulation *simulation) {
    simulation_ = simulation;
}

void RunLoop::registerCallback(float time, RunLoopCallback *callback) {
    Item item;
    item.time = time;
    item.callback = callback;
    queue_.push(item);
}

bool RunLoop::Item::operator>(const Item &other) const {
    return time > other.time;
}
