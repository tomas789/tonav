//
// Created by Tomas Krejci on 10/8/17.
//

#include "trajectory/circular_trajectory.h"

#include <cmath>

std::unique_ptr<Trajectory> CircularTrajectory::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<CircularTrajectory> trajectory(new CircularTrajectory(sim_setup));
    
    trajectory->radius_ = j.at("radius");
    trajectory->time_per_revolution_ = j.at("time_per_revolution");
    
    return std::move(trajectory);
}

void CircularTrajectory::initialize(VioSimulation *simulation) {

}

Eigen::Vector3d CircularTrajectory::getBodyPositionInGlobalFrame(double time) const {
    double t = M_2_PI*time/time_per_revolution_;
    double s = std::sin(t);
    double c = std::cos(t);
    Eigen::Vector3d position;
    position << radius_*s, radius_*c, 0;
    return position;
}

tonav::Quaternion CircularTrajectory::getGlobalToBodyFrameRotation(double time) const {
    double theta = M_2_PI*time/time_per_revolution_;
    return tonav::Quaternion(0, 0, std::sin(theta/2), std::cos(theta/2)).conjugate();
}

Eigen::Vector3d CircularTrajectory::getGlobalGravity() const {
    Eigen::Vector3d global_gravity;
    global_gravity << 0, 0, -9.81;
    return global_gravity;
}

CircularTrajectory::~CircularTrajectory() = default;

CircularTrajectory::CircularTrajectory(SimSetup *sim_setup)
    : Trajectory(sim_setup) {
    
}

