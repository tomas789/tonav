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
    
    virtual Eigen::Vector3d getCameraPositionInGlobalFrame(double time) const = 0;
    virtual tonav::Quaternion getGlobalToCameraFrameRotation(double time) const = 0;
    
    virtual Eigen::Vector3d getCameraPositionInBodyFrame() const = 0;
    virtual tonav::Quaternion getBodyToCameraFrameRotation() const = 0;
    
    virtual ~Trajectory();

protected:
    Trajectory(SimSetup *sim_setup);
};

#endif //TONAV_TRAJECTORY_H
