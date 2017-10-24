//
// Created by Tomas Krejci on 10/22/17.
//

#ifndef TONAV_ODOMETRY_H
#define TONAV_ODOMETRY_H

#include <json.hpp>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <tonav.h>

#include "sim_setup_component.h"

using json = nlohmann::json;

class Odometry: public SimSetupComponent {
public:
    static std::unique_ptr<Odometry> load(SimSetup *sim_setup, const json& j);
    
    virtual void initialize(VioSimulation *simulation) = 0;
    
    virtual void updateAcceleration(double time, const Eigen::Vector3d& accel) = 0;
    virtual void updateRotationRate(double time, const Eigen::Vector3d& gyro) = 0;
    virtual void updateFrame(double time, const cv::Mat& frame) = 0;
    
    virtual Eigen::Vector3d getBodyPositionInGlobalFrame() const = 0;
    virtual tonav::Quaternion getGlobalToBodyFrameRotation() const = 0;
    
    virtual Eigen::Vector3d getCameraPositionInGlobalFrame() const = 0;
    virtual tonav::Quaternion getGlobalToCameraFrameRotation() const = 0;
    
    virtual ~Odometry();
    
protected:
    Odometry(SimSetup *sim_setup);
};

#endif //TONAV_ODOMETRY_H
