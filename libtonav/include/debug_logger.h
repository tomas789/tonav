//
// Created by Tomas Krejci on 11/2/17.
//

#ifndef TONAV_DEBUG_LOGGER_H
#define TONAV_DEBUG_LOGGER_H

#include <json.hpp>

using nlohmann::json;

namespace tonav {

class FeatureId;

class DebugLogger {
private:
    class LoggerNode {
    public:
        friend class DebugLogger;
        
        void log(const std::string& key, const std::string& message);
        
    private:
        std::vector<std::pair<std::string, std::string>> messages_;
        
        json getJsonRepresentation() const;
    };
    
public:
    void setOutputFile(const std::string& output_file);
    LoggerNode& getFeatureNode(const FeatureId& feature_id);
    
    void setUseTerminalExceptionWriter();
    
    static DebugLogger& getInstance();
    
    
private:
    std::string output_file_;
    std::string termination_reason_;
    std::map<FeatureId, LoggerNode> feature_nodes_;
    
    static DebugLogger instance_;
    
    DebugLogger() = default;
    ~DebugLogger();
    
    json getJsonRepresentation() const;
    void write() const;
};

}

#endif //TONAV_DEBUG_LOGGER_H
