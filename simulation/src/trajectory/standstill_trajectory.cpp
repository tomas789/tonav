//
// Created by Tomas Krejci on 10/24/17.
//

#include "trajectory/standstill_trajectory.h"

std::unique_ptr<StandstillTrajectory> StandstillTrajectory::load(SimSetup *sim_setup, const json& j) {
    std::unique_ptr<StandstillTrajectory> trajectory(new StandstillTrajectory(sim_setup));
    
    return trajectory;
}

void StandstillTrajectory::initialize(VioSimulation *simulation) {
    // Do nothing.
}

Eigen::Vector3d StandstillTrajectory::getBodyPositionInGlobalFrame(double time) const {
    return Eigen::Vector3d::Zero();
}

tonav::Quaternion StandstillTrajectory::getGlobalToBodyFrameRotation(double time) const {
    return tonav::Quaternion::identity();
}

Eigen::Vector3d StandstillTrajectory::getGlobalGravity() const {
    Eigen::Vector3d global_gravity;
    global_gravity << 0, 0, -9.81;
    return global_gravity;
}

StandstillTrajectory::~StandstillTrajectory() = default;

StandstillTrajectory::StandstillTrajectory(SimSetup *sim_setup) : Trajectory(sim_setup) {
    // Do nothing.
}
