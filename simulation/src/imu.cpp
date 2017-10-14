//
// Created by Tomas Krejci on 10/7/17.
//

#include "imu.h"
#include "imu/numerical_diff_imu.h"

std::unique_ptr<Imu> Imu::load(SimSetup *sim_setup, const json &j) {
    auto& type = j.at("type");
    if (type == "numerical_diff") {
        return NumericalDiffImu::load(sim_setup, j.at("params"));
    } else {
        throw "Unknown imu type.";
    }
}

float Imu::getUpdateFrequency() const {
    return update_frequency_;
}

Imu::Imu(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }
Imu::~Imu() = default;