#ifndef TONAV_STATS_TIMER_H
#define TONAV_STATS_TIMER_H

#include <chrono>
#include <string>

class StatsTimer {
public:
    StatsTimer(const std::string& key);
    ~StatsTimer();
protected:
    std::chrono::time_point<std::chrono::high_resolution_clock> begin_;
    std::string key_;
};

#endif //TONAV_STATS_TIMER_H
