//
// Created by Tomas Krejci on 10/7/17.
//

#include "imu.h"
#include "imu/numerical_diff_imu.h"

std::unique_ptr<Imu> Imu::load(SimSetup *sim_setup, const json &j) {
    std::string type = j.at("type").get<std::string>();
    json params = j.at("params");
    
    std::unique_ptr<Imu> imu;
    
    if (type == "numerical_diff") {
        imu = NumericalDiffImu::load(sim_setup, params);
    } else {
        throw std::runtime_error("Unknown imu type.");
    }
    
    imu->update_frequency_ = params.at("update_frequency").get<double>();
    if (imu->update_frequency_ <= 0) {
        throw std::runtime_error("'update_frequency' has to be a positive number.");
    }
    
    return imu;
}

double Imu::getUpdateFrequency() const {
    return update_frequency_;
}

Imu::Imu(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }
Imu::~Imu() = default;
