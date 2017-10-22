//
// Created by Tomas Krejci on 10/22/17.
//

#include "odometry.h"
#include "odometry/tonav_body_state_odometry.h"

std::unique_ptr<Odometry> Odometry::load(SimSetup *sim_setup, const json& j) {
    std::string type = j.at("type").get<std::string>();
    json params = j.at("params");
    
    std::unique_ptr<Odometry> odometry;
    
    if (type == "tonav_body_state") {
        odometry = std::move(TonavBodyStateOdometry::load(sim_setup, params));
    } else {
        throw std::runtime_error("Unknown odometry type.");
    }
    
    return odometry;
}

Odometry::~Odometry() = default;

Odometry::Odometry(SimSetup *sim_setup) : SimSetupComponent(sim_setup) {

}
