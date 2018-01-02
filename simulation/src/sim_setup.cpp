//
// Created by Tomas Krejci on 10/7/17.
//

#include "sim_setup.h"

#include <json.hpp>

#include "imu.h"
#include "trajectory.h"
#include "helpers/kitti_loader_helper.h"

using json = nlohmann::json;

std::map<std::string, SimSetup::ComponentFactory<Helper>> SimSetup::registered_helper_;
std::map<std::string, SimSetup::ComponentFactory<Trajectory>> SimSetup::registered_trajectory_;
std::map<std::string, SimSetup::ComponentFactory<Imu>> SimSetup::registered_imu_;
std::map<std::string, SimSetup::ComponentFactory<Vision>> SimSetup::registered_vision_;
std::map<std::string, SimSetup::ComponentFactory<Odometry>> SimSetup::registered_odometry_;

std::shared_ptr<SimSetup> SimSetup::load(std::string filename) {
    SimSetup::registerDefaultComponents();
    std::shared_ptr<SimSetup> sim_setup(new SimSetup);
    
    std::ifstream file_stream(filename);
    if (!file_stream.is_open()) {
        throw std::runtime_error("Cannot open sim_setup file.");
    }
    json j;
    file_stream >> j;
    
    if (j.find("helpers") != std::end(j) && j["helpers"].is_array()) {
        for (const auto& j_helper : j["helpers"]) {
            std::string helper_type = j_helper["type"];
            auto it = SimSetup::registered_helper_.find(helper_type);
            if (it == std::end(SimSetup::registered_helper_)) {
                throw std::runtime_error("Trying to load unsupported helper type.");
            }
            std::unique_ptr<Helper> helper = it->second(sim_setup.get(), j_helper["params"]);
            std::string helper_name = j_helper["id"];
            sim_setup->helpers_.emplace(helper_name, std::move(helper));
        }
    }
    
    if (j.find("trajectory") != std::end(j)) {
        sim_setup->trajectory_ = Trajectory::load(sim_setup.get(), j.at("trajectory"));
    }
    
    if (j.find("imu") != std::end(j)) {
        sim_setup->imu_ = Imu::load(sim_setup.get(), j.at("imu"));
    }
    
    if (j.find("vision") != std::end(j)) {
        sim_setup->vision_ = Vision::load(sim_setup.get(), j.at("vision"));
    }
    
    if (j.find("odometry") != std::end(j)) {
        sim_setup->odometry_ = Odometry::load(sim_setup.get(), j.at("odometry"));
    }
    
    if (j.find("trajectory") != std::end(j)) {
        sim_setup->trajectory_ = Trajectory::load(sim_setup.get(), j.at("trajectory"));
    }
    
    return sim_setup;
}

void SimSetup::registerDefaultComponents() {
    SimSetup::registerHelper<KittiLoaderHelper>();
}

bool SimSetup::hasHelper(const std::string &name) const {
    return helpers_.find(name) != std::end(helpers_);
}

Helper& SimSetup::getHelper(const std::string &name) {
    return *helpers_.at(name);
}

const Helper& SimSetup::getHelper(const std::string &name) const {
    return *helpers_.at(name);
}

std::vector<std::string> SimSetup::getAllHelperNames() const {
    std::vector<std::string> all_names;
    for (const auto& it : helpers_) {
        all_names.push_back(it.first);
    }
    return all_names;
}

Trajectory& SimSetup::getTrajectory() {
    return *trajectory_;
}

const Trajectory& SimSetup::getTrajectory() const {
    return *trajectory_;
}

void SimSetup::setTrajectory(std::unique_ptr<Trajectory> &&trajectory) {
    trajectory_ = std::move(trajectory);
}

Imu& SimSetup::getImu() {
    return *imu_;
}

const Imu& SimSetup::getImu() const {
    return *imu_;
}

void SimSetup::setImu(std::unique_ptr<Imu>&& imu) {
    imu_ = std::move(imu);
}

Vision& SimSetup::getVision() {
    return *vision_;
}

const Vision& SimSetup::getVision() const {
    return *vision_;
}

void SimSetup::setVision(std::unique_ptr<Vision> &&vision) {
    vision_ = std::move(vision);
}

Odometry& SimSetup::getOdometry() {
    return *odometry_;
}

const Odometry& SimSetup::getOdometry() const {
    return *odometry_;
}

void SimSetup::setOdometry(std::unique_ptr<Odometry> &&odometry) {
    odometry_ = std::move(odometry);
}

SimSetup::SimSetup() = default;
