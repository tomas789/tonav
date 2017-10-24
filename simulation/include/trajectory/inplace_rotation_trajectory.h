//
// Created by Tomas Krejci on 10/24/17.
//

#ifndef TONAV_INPLACE_ROTATION_TRAJECTORY_H
#define TONAV_INPLACE_ROTATION_TRAJECTORY_H

#include "../trajectory.h"

class InplaceRotationTrajectory: public Trajectory {
public:
    static std::unique_ptr<InplaceRotationTrajectory> load(SimSetup *sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    Eigen::Vector3d getBodyPositionInGlobalFrame(double time) const;
    tonav::Quaternion getGlobalToBodyFrameRotation(double time) const;
    
    Eigen::Vector3d getCameraPositionInGlobalFrame(double time) const;
    tonav::Quaternion getGlobalToCameraFrameRotation(double time) const;
    
    Eigen::Vector3d getCameraPositionInBodyFrame() const;
    tonav::Quaternion getBodyToCameraFrameRotation() const;
    
    Eigen::Vector3d getGlobalGravity() const;
    
    virtual ~InplaceRotationTrajectory();

private:
    InplaceRotationTrajectory(SimSetup *sim_setup);
    
    double seconds_per_revolution_;
    Eigen::Vector3d axis_of_revolution_;
};

#endif //TONAV_INPLACE_ROTATION_TRAJECTORY_H
