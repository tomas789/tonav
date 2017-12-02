//
// Created by Tomas Krejci on 11/3/17.
//

#include "debug_logger.h"

#include <exception>
#include <fstream>

#include "feature_id.h"

namespace tonav {

DebugLogger DebugLogger::instance_;

void DebugLogger::LoggerNode::logImpl(const std::string& key, const std::string& message) {
    messages_.emplace_back(key, message);
}

json DebugLogger::LoggerNode::getJsonRepresentation() const {
    json representation;
    for (auto it = std::begin(messages_); it != std::end(messages_); ++it) {
        representation[it->first] = it->second;
    }
    return representation;
}

void DebugLogger::setOutputFile(const std::string &output_file) {
    std::cout << "Setting output file to " << output_file << std::endl;
    output_file_ = output_file;
}
    
void DebugLogger::writeAndClear() {
    write();
    termination_reason_ = "";
    feature_nodes_.clear();
}

DebugLogger::LoggerNode &DebugLogger::getFeatureNode(const FeatureId &feature_id) {
    return feature_nodes_[feature_id];
}

void DebugLogger::setUseTerminalExceptionWriter() {
    std::set_terminate([]() {
        std::exception_ptr e = std::current_exception();
        if (!e) {
            std::abort();
        }
        try {
            std::rethrow_exception(e);
        } catch(const std::exception& e) {
            DebugLogger::getInstance().termination_reason_ = e.what();
            DebugLogger::getInstance().write();
            std::abort();
        }
    });
}

DebugLogger& DebugLogger::getInstance() {
    return DebugLogger::instance_;
}

DebugLogger::~DebugLogger() {
    write();
}

json DebugLogger::getJsonRepresentation() const {
    json representation = json::object();
    
    if (!feature_nodes_.empty()) {
        json features_json;
        for (auto it = std::begin(feature_nodes_); it != std::end(feature_nodes_); ++it) {
            const FeatureId &feature_id = it->first;
            const LoggerNode &node = it->second;
            std::string feature_desc = "feature_" + std::to_string(feature_id.getFrameId()) + "_" + std::to_string(feature_id.getFeatureId());
            features_json[feature_desc] = node.getJsonRepresentation();
        }
        representation["features"] = features_json;
    }
    
    if (!termination_reason_.empty()) {
        representation["termination_reason"] = termination_reason_;
    }
    
    return representation;
}

void DebugLogger::write() const {
    std::cout << "writing debug output to " << output_file_ << std::endl;
    std::ofstream out(output_file_.empty() ? "debug_output.log" : output_file_);
    try {
        if (out) {
            out << getJsonRepresentation().dump(4);
        } else {
            std::cerr << "Cannot write DebugLogger output. Cannot open file." << std::endl;
        }
    } catch(const std::exception& e) {
        std::cout << e.what() << std::endl;
    }
}

} // namespace tonav
