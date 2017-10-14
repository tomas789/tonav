#ifndef TONAV_STATS_H
#define TONAV_STATS_H

#include <string>

#include "stats_node.h"

namespace tonav {

class Stats {
public:
    Stats();
    
    static Stats &getGlobalInstance();
    
    StatsNode &operator[](std::string key);
    
    StatsNode &current();
    
    void openLevel(const std::string &name);
    
    void closeCurrentLevel();
    
    std::string str() const;

protected:
    static std::unique_ptr<Stats> instance_;
    
    StatsNode stats_tree_;
    StatsNode *current_node_;
};

}

#endif //TONAV_STATS_H
