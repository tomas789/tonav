//
// Created by Tomas Krejci on 10/22/17.
//

#ifndef TONAV_TONAV_BODY_STATE_ODOMETRY_H
#define TONAV_TONAV_BODY_STATE_ODOMETRY_H

#include <json.hpp>
#include <tonav.h>

#include "../odometry.h"
#include "tonav_calibration.h"

using json = nlohmann::json;

class VioSimulation;

class TonavBodyStateOdometry: public Odometry {
public:
    static std::unique_ptr<TonavBodyStateOdometry> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    void updateAcceleration(double time, const Eigen::Vector3d& accel);
    void updateRotationRate(double time, const Eigen::Vector3d& gyro);
    void updateFrame(double time, const cv::Mat& frame);
    
    Eigen::Vector3d getBodyPositionInGlobalFrame();
    tonav::Quaternion getGlobalToBodyFrameRotation();
    
    Eigen::Vector3d getCameraPositionInGlobalFrame();
    tonav::Quaternion getGlobalToCameraFrameRotation();
    
    virtual ~TonavBodyStateOdometry();
    
private:
    TonavBodyStateOdometry(SimSetup *sim_setup);
    
    void propagateBodyState();
    
    VioSimulation *vio_simulation_;
    
    double next_propagation_time_;
    Eigen::Vector3d next_accel_;
    Eigen::Vector3d next_gyro_;
    
    std::shared_ptr<TonavCalibration> tonav_calibration_;
    std::shared_ptr<tonav::BodyState> tonav_body_state_;
};

#endif //TONAV_TONAV_BODY_STATE_ODOMETRY_H
