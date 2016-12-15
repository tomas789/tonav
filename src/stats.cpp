#include "stats.h"

std::unique_ptr<Stats> Stats::instance_;

Stats& Stats::getGlobalInstance() {
    if (!Stats::instance_) {
        Stats::instance_.reset(new Stats);
    }
    return *Stats::instance_;
}

StatsNode& Stats::operator[](std::string key) {
    return stats_tree_[key];
}

StatsNode& Stats::current() {
    return *current_node_;
}

Stats::Stats() : stats_tree_(nullptr) {
    current_node_ = &stats_tree_;
}

std::string Stats::str() const {
    return stats_tree_.str();
}

void Stats::openLevel(const std::string& name) {
    current_node_ = std::addressof((*current_node_)[name]);
}

void Stats::closeCurrentLevel() {
    current_node_ = current_node_->parent_;
}
