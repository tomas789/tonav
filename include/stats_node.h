#ifndef TONAV_STATS_NODE_H
#define TONAV_STATS_NODE_H

#include <string>
#include <map>
#include <vector>

class Stats;
class StatsTimer;

class StatsNode {
public:
    friend class Stats;
    
    StatsNode(StatsNode* parent);
    
    StatsNode& operator=(const std::string& value);
    StatsNode& operator=(const StatsNode& other);
    
    template <typename T>
    void add(const T& item) {
        nodes_.clear();
        terminal_items_.push_back(std::to_string(item));
    }
    
    StatsNode& operator[](const std::string& key);
    const StatsNode& operator[](const std::string& key) const;
    
    bool is_terminal() const;
    
    std::string str(int indent = 0) const;
protected:
    static const int indent_size = 4;
    StatsNode* parent_;
    std::vector<std::string> terminal_items_;
    std::map<std::string, StatsNode> nodes_;
};

#endif //TONAV_STATS_NODE_H
