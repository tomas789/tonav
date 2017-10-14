//
// Created by Tomas Krejci on 10/7/17.
//

#include "sim_setup.h"

#include <json.hpp>

#include "imu.h"
#include "trajectory.h"

using json = nlohmann::json;

std::shared_ptr<SimSetup> SimSetup::load(std::string filename) {
    std::shared_ptr<SimSetup> sim_setup(new SimSetup);
    
    std::ifstream file_stream(filename);
    if (!file_stream.is_open()) {
        throw "Cannot open sim_setup file.";
    }
    json j;
    file_stream >> j;
    
    std::unique_ptr<Trajectory> trajectory = Trajectory::load(sim_setup.get(), j.at("trajectory"));
    if (!trajectory)
        throw "Cannot load trajectory.";
    sim_setup->trajectory_ = std::move(trajectory);
    
    std::unique_ptr<Imu> imu = Imu::load(sim_setup.get(), j.at("imu"));
    if (!imu)
        throw "Cannot load imu.";
    sim_setup->imu_ = std::move(imu);
    
    std::unique_ptr<Vision> vision = Vision::load(sim_setup.get(), j.at("vision"));
    if (!vision)
        throw "Cannot load vision.";
    sim_setup->vision_ = std::move(vision);
    
    return sim_setup;
}

Trajectory& SimSetup::getTrajectory() {
    return *trajectory_;
}

const Trajectory& SimSetup::getTrajectory() const {
    return *trajectory_;
}

Imu& SimSetup::getImu() {
    return *imu_;
}

const Imu& SimSetup::getImu() const {
    return *imu_;
}

Vision& SimSetup::getVision() {
    return *vision_;
}

const Vision& SimSetup::getVision() const {
    return *vision_;
}

SimSetup::SimSetup() = default;