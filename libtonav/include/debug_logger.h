//
// Created by Tomas Krejci on 11/2/17.
//

#ifndef TONAV_DEBUG_LOGGER_H
#define TONAV_DEBUG_LOGGER_H

#include <json.hpp>
#include <Eigen/Core>
#include <sstream>

using nlohmann::json;

namespace tonav {

class FeatureId;

class DebugLogger {
private:
    class LoggerNode {
    public:
        friend class DebugLogger;
        
        template <typename Derived>
        void logEigen(const std::string& key, const Eigen::MatrixBase<Derived>& mat) {
            Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
            std::ostringstream ss;
            ss << mat.format(CommaInitFmt);
            logImpl(key, ss.str());
        }
    
        template <typename T>
        void log(const std::string& key, const T& message) {
            logImpl(key, std::to_string(message));
        }


    private:
        std::vector<std::pair<std::string, std::string>> messages_;
    
        void logImpl(const std::string& key, const std::string& message);
        
        json getJsonRepresentation() const;
    };
    
public:
    void setOutputFile(const std::string& output_file);
    void writeAndClear();
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
