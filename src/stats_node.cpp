#include "stats_node.h"

#include <iterator>
#include <cassert>
#include <sstream>

StatsNode::StatsNode(StatsNode* parent) {
    parent_ = parent;
}

StatsNode& StatsNode::operator=(const std::string& value) {
    terminal_items_.clear();
    terminal_items_.push_back(value);
    nodes_.clear();
    return *this;
}

StatsNode& StatsNode::operator=(const StatsNode& other) {
    nodes_ = other.nodes_;
    parent_ = other.parent_;
    return *this;
}

StatsNode& StatsNode::operator[](const std::string &key) {
    auto it = nodes_.find(key);
    if (it == std::end(nodes_)) {
        auto it_new = nodes_.insert(std::make_pair(key, StatsNode(this)));
        return it_new.first->second;
    }
    return it->second;
}

bool StatsNode::is_terminal() const {
    return !(terminal_items_.empty());
}

std::string StatsNode::str(int indent) const {
    if (is_terminal()) {
        std::ostringstream out;
        
        if (terminal_items_.size() == 1) {
            out << "\"" << terminal_items_.front() << "\"";
        } else {
            out << "[";
            for (std::size_t i = 0; i < terminal_items_.size() - 1; ++i) {
                out << "\"" << terminal_items_[i] << "\", ";
            }
            out << "\"" << terminal_items_.back() << "\"]";
        }
        return out.str();
    }
    
    std::ostringstream out;
    std::string indent_str(indent*indent_size, ' ');
    std::string indent_deep_str((indent + 1)*indent_size, ' ');
    out << "{" << std::endl;
    for (auto it = std::begin(nodes_); it != std::end(nodes_); ++it) {
        out << indent_deep_str << "\"" << it->first << "\": " << it->second.str(indent + 1);
        if (std::distance(std::begin(nodes_), it) < nodes_.size() - 1) {
            out << ",";
        }
        out << std::endl;
    }
    out << indent_str << "}";
    return out.str();
}
