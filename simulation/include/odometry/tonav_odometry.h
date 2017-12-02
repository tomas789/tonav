//
// Created by Tomas Krejci on 10/28/17.
//

#ifndef TONAV_TONAV_ODOMETRY_H
#define TONAV_TONAV_ODOMETRY_H

#include <json.hpp>
#include <tonav.h>

#include "../custom_serialization.h"
#include "../odometry.h"
#include "tonav_calibration.h"

using json = nlohmann::json;

class VioSimulation;

class TonavOdometry: public Odometry {
public:
    static std::unique_ptr<TonavOdometry> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    void updateAcceleration(double time, const Eigen::Vector3d& accel);
    void updateRotationRate(double time, const Eigen::Vector3d& gyro);
    void updateAcceleration(double time, const Eigen::Vector3d& accel, bool& was_updated);
    void updateRotationRate(double time, const Eigen::Vector3d& gyro, bool& was_updated);
    void updateFrame(double time, const cv::Mat& frame);
    
    Eigen::Vector3d getBodyPositionInGlobalFrame() const;
    tonav::Quaternion getGlobalToBodyFrameRotation() const;
    
    Eigen::Vector3d getCameraPositionInGlobalFrame() const;
    tonav::Quaternion getGlobalToCameraFrameRotation() const;
    
    std::shared_ptr<tonav::Tonav> getTonav();
    std::shared_ptr<TonavCalibration> getTonavCalibration();
    
    virtual ~TonavOdometry();

private:
    TonavOdometry(SimSetup *sim_setup);
    
    void updateTonavInitializerFromGroundTruth(double time);
    void evaluateToGroundTruth(double time);
    
    void writeEvalGt(json& j, const std::string& key, const Eigen::Vector2d& true_state, const Eigen::Vector2d& estimate);
    void writeEvalGt(json& j, const std::string& key, const Eigen::Vector3d& true_state, const Eigen::Vector3d& estimate);
    void writeEvalGt(json& j, const std::string& key, const tonav::Quaternion& true_state, const tonav::Quaternion& estimate);
    void writeEvalGt(json& j, const std::string& key, double true_state, double estimate);
    
    VioSimulation *vio_simulation_;
    
    double next_propagation_time_;
    Eigen::Vector3d next_accel_;
    Eigen::Vector3d next_gyro_;
    
    std::shared_ptr<TonavCalibration> tonav_calibration_;
    std::shared_ptr<tonav::Tonav> tonav_;
    
    json eval_to_gt_;
};

#endif //TONAV_TONAV_ODOMETRY_H
