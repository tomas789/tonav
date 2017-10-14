//
// Created by Tomas Krejci on 10/8/17.
//

#include "trajectory/circular_trajectory.h"

#include <cmath>

std::unique_ptr<Trajectory> CircularTrajectory::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<CircularTrajectory> trajectory(new CircularTrajectory(sim_setup));
    
    json radius_json = j.at("radius");
    if (!radius_json.is_number())
        throw "'radius' has to be a number.";
    trajectory->radius_ = radius_json;
    
    json time_per_revolution_json = j.at("time_per_revolution");
    if (!time_per_revolution_json.is_number())
        throw "'time_per_revolution' has to be a number.";
    trajectory->time_per_revolution_ = time_per_revolution_json;
    
    return std::move(trajectory);
}

void CircularTrajectory::initialize(VioSimulation *simulation) {

}

Eigen::Vector3d CircularTrajectory::getBodyPositionInGlobalFrame(double time) {
    double t = M_2_PI*time/time_per_revolution_;
    double s = std::sin(t);
    double c = std::cos(t);
    Eigen::Vector3d position;
    position << radius_*s, radius_*c, 0;
    return position;
}

tonav::Quaternion CircularTrajectory::getGlobalToBodyFrameRotation(double time) {
    double theta = M_2_PI*time/time_per_revolution_;
    return tonav::Quaternion(0, 0, std::sin(theta/2), std::cos(theta/2)).conjugate();
}

CircularTrajectory::~CircularTrajectory() = default;

CircularTrajectory::CircularTrajectory(SimSetup *sim_setup) : Trajectory(sim_setup) { }

