//
// Created by Tomas Krejci on 10/8/17.
//

#include "trajectory.h"

#include <limits>

#include "trajectory/circular_trajectory.h"
#include "trajectory/standstill_trajectory.h"

std::unique_ptr<Trajectory> Trajectory::load(SimSetup *sim_setup, const json &j) {
    std::string type = j.at("type").get<std::string>();
    json params = j.at("params");
    
    std::unique_ptr<Trajectory> trajectory;
    
    if (type == "circular") {
        trajectory = std::move(CircularTrajectory::load(sim_setup, params));
    } else if (type == "standstill") {
        trajectory = std::move(StandstillTrajectory::load(sim_setup, params));
    } else {
        throw std::runtime_error("Unknown trajectory type.");
    }
    
    return std::move(trajectory);
}

Trajectory::~Trajectory() = default;

Trajectory::Trajectory(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }