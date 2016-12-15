#include "stats_timer.h"

#include "stats.h"

StatsTimer::StatsTimer(const std::string& key) {
    key_ = key;
    begin_ = std::chrono::high_resolution_clock::now();
}

StatsTimer::~StatsTimer() {
    auto end = std::chrono::high_resolution_clock::now();
    Stats& stats = Stats::getGlobalInstance();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end-begin_).count();
    stats.current()[key_].add(duration);
}
