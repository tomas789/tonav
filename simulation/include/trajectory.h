//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_TRAJECTORY_H
#define TONAV_TRAJECTORY_H

#include <Eigen/Core>
#include <tonav.h>
#include <json.hpp>

#include "sim_setup_component.h"

class SimSetup;

using json = nlohmann::json;

class Trajectory: protected SimSetupComponent {
public:
    static std::unique_ptr<Trajectory> load(SimSetup *sim_data, const json& j);
    
    virtual void initialize(VioSimulation *simulation) = 0;
    
    virtual Eigen::Vector3d getBodyPositionInGlobalFrame(double time) const = 0;
    virtual tonav::Quaternion getGlobalToBodyFrameRotation(double time) const = 0;
    
    Eigen::Vector3d getCameraPositionInGlobalFrame(double time) const;
    tonav::Quaternion getGlobalToCameraFrameRotation(double time) const;
    
    Eigen::Vector3d getCameraPositionInBodyFrame() const;
    tonav::Quaternion getBodyToCameraFrameRotation() const;
    
    virtual Eigen::Vector3d getGlobalGravity() const = 0;
    
    virtual ~Trajectory();

protected:
    Trajectory(SimSetup *sim_setup);
    
    tonav::Quaternion q_C_B_;
    Eigen::Vector3d p_C_B_;
};

#endif //TONAV_TRAJECTORY_H
