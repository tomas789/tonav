//
// Created by Tomas Krejci on 10/14/17.
//

#include "vision.h"

#include "vision/generated_features_vision.h"

std::unique_ptr<Vision> Vision::load(SimSetup *sim_setup, const json& j) {
    std::string type = j.at("type").get<std::string>();
    json params = j.at("params");
    
    std::unique_ptr<Vision> vision;
    
    if (type == "generated_features") {
        vision = GeneratedFeaturesVision::load(sim_setup, params);
    } else {
        throw std::runtime_error("Unknown vision type.");
    }
    
    vision->update_frequency_ = params.at("update_frequency").get<double>();
    if (vision->update_frequency_ <= 0) {
        throw std::runtime_error("'update_frequency' has to be a positive number.");
    }
    
    return vision;
}

double Vision::getUpdateFrequency() const {
    return update_frequency_;
}

Vision::~Vision() = default;
Vision::Vision(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }
