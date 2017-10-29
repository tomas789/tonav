//
// Created by Tomas Krejci on 10/24/17.
//

#include "trajectory/inplace_rotation_trajectory.h"

#include "custom_serialization.h"

std::unique_ptr<InplaceRotationTrajectory> InplaceRotationTrajectory::load(SimSetup *sim_setup, const json& j) {
    std::unique_ptr<InplaceRotationTrajectory> trajectory(new InplaceRotationTrajectory(sim_setup));
    
    trajectory->seconds_per_revolution_ = j.at("seconds_per_revolution");
    trajectory->axis_of_revolution_ = j.at("axis_of_revolution");
    
    return std::move(trajectory);
}

void InplaceRotationTrajectory::initialize(VioSimulation *simulation) {
    axis_of_revolution_ = axis_of_revolution_.normalized();
}

Eigen::Vector3d InplaceRotationTrajectory::getBodyPositionInGlobalFrame(double time) const {
    return Eigen::Vector3d::Zero();
}

tonav::Quaternion InplaceRotationTrajectory::getGlobalToBodyFrameRotation(double time) const {
    double revolution = time/seconds_per_revolution_;
    double theta = M_2_PI*revolution;
    
    double s = std::sin(theta/2);
    double c = std::cos(theta/2);
    
    tonav::Quaternion q_B_G(s*axis_of_revolution_(0), s*axis_of_revolution_(1), s*axis_of_revolution_(2), c);
    assert(std::abs(q_B_G.norm() - 1) < 1e-8);
    return q_B_G;
}

Eigen::Vector3d InplaceRotationTrajectory::getGlobalGravity() const {
    Eigen::Vector3d global_gravity;
    global_gravity << 0, 0, -9.81;
    return global_gravity;
}

InplaceRotationTrajectory::~InplaceRotationTrajectory() = default;

InplaceRotationTrajectory::InplaceRotationTrajectory(SimSetup *sim_setup) : Trajectory(sim_setup) {
    // Do nothing.
}
