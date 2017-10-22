//
// Created by Tomas Krejci on 10/8/17.
//

#ifndef TONAV_CIRCULAR_TRAJECTORY_H
#define TONAV_CIRCULAR_TRAJECTORY_H

#include <json.hpp>

#include "../trajectory.h"

class VioSimulation;

using json = nlohmann::json;

class CircularTrajectory: public Trajectory {
public:
    static std::unique_ptr<Trajectory> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    Eigen::Vector3d getBodyPositionInGlobalFrame(double time) const;
    tonav::Quaternion getGlobalToBodyFrameRotation(double time) const;
    
    Eigen::Vector3d getCameraPositionInGlobalFrame(double time) const;
    tonav::Quaternion getGlobalToCameraFrameRotation(double time) const;
    
    Eigen::Vector3d getCameraPositionInBodyFrame() const;
    tonav::Quaternion getBodyToCameraFrameRotation() const;
    
    virtual ~CircularTrajectory();

protected:
    CircularTrajectory(SimSetup *sim_setup);
    
    double radius_;
    double time_per_revolution_;
    
    tonav::Quaternion q_C_B_;
    Eigen::Vector3d p_C_B_;
};

#endif //TONAV_CIRCULAR_TRAJECTORY_H
