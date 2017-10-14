//
// Created by Tomas Krejci on 10/8/17.
//

#include "trajectory.h"

#include <limits>

#include "trajectory/circular_trajectory.h"

std::unique_ptr<Trajectory> Trajectory::load(SimSetup *sim_setup, const json &j) {
    auto& type = j.at("type");
    if (type == "circular") {
        return CircularTrajectory::load(sim_setup, j.at("params"));
    } else {
        throw "Unknown imu type.";
    }
}

Trajectory::~Trajectory() = default;

Trajectory::Trajectory(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }