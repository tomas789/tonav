//
// Created by Tomas Krejci on 10/14/17.
//

#include "vision.h"

#include "vision/generated_features_vision.h"

std::unique_ptr<Vision> Vision::load(SimSetup *sim_setup, const json& j) {
    auto& type = j.at("type");
    if (type == "generated_features") {
        return GeneratedFeaturesVision::load(sim_setup, j.at("params"));
    } else {
        throw "Unknown imu type.";
    }
}

Vision::~Vision() = default;
Vision::Vision(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }
