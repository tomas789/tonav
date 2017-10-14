//
// Created by Tomas Krejci on 10/7/17.
//

#include "imu/numerical_diff_imu.h"

#include "sim_setup.h"
#include "vio_simulation.h"

std::unique_ptr<Imu> NumericalDiffImu::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<NumericalDiffImu> imu(new NumericalDiffImu(sim_setup));
    
    json diff_time_delta_json = j.at("diff_time_delta");
    if (!diff_time_delta_json.is_number())
        throw "'diff_time_delta' has to be a number.";
    imu->diff_time_delta_ = diff_time_delta_json;
    
    json update_frequency_json = j.at("update_frequency");
    if (!update_frequency_json.is_number())
        throw "'update_frequency' has to be a number.";
    imu->update_frequency_ = update_frequency_json;
    if (imu->update_frequency_ <= 0.0)
        throw "'update_frequency' has to be positive number.";
    
    return std::move(imu);
}

void NumericalDiffImu::initialize(VioSimulation *simulation) {
    simulation_ = simulation;
    simulation_->getRunLoop().registerCallback(0, this);
}

void NumericalDiffImu::runLoopCallback(double time) {
    Eigen::Vector3d accel = getAccelerometerData(time);
    Eigen::Vector3d gyro = getGyroscopeData(time);
    
    simulation_->accelerometerCallback(time, accel);
    simulation_->gyroscopeCallback(time, gyro);
    
    simulation_->getRunLoop().registerCallback(time+1.0/update_frequency_, this);
}

Eigen::Vector3d NumericalDiffImu::getAccelerometerData(double time) const {
    std::function<Eigen::Vector3d(double)> f = [&](double t) {
        return getVelocity(t);
    };
    Eigen::Vector3d a_g = diff(f, time, diff_time_delta_);
    Eigen::Matrix3d R_B_G = sim_setup_->getTrajectory().getGlobalToBodyFrameRotation(time).toRotationMatrix();
    Eigen::Vector3d a_b = R_B_G*(a_g + global_gravity_);
    Eigen::Matrix3d T_a = Eigen::Matrix3d::Identity();
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_m = T_a*a_b + b_a;
    return a_m;
}

Eigen::Vector3d NumericalDiffImu::getGyroscopeData(double time) const {
    return Eigen::Vector3d::Zero();
}

NumericalDiffImu::~NumericalDiffImu() = default;

NumericalDiffImu::NumericalDiffImu(SimSetup *sim_setup) : Imu(sim_setup) {
    global_gravity_ << 0, 0, 9.81;
}

Eigen::Vector3d NumericalDiffImu::getVelocity(double time) const {
    std::function<Eigen::Vector3d(double)> f = [&](double t) {
        return sim_setup_->getTrajectory().getBodyPositionInGlobalFrame(t);
    };
    return diff(f, time, diff_time_delta_);
}
