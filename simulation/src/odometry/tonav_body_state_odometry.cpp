//
// Created by Tomas Krejci on 10/22/17.
//

#include "odometry/tonav_body_state_odometry.h"

#include "vio_simulation.h"

std::unique_ptr<TonavBodyStateOdometry> TonavBodyStateOdometry::load(SimSetup* sim_setup, const json& j) {
    std::unique_ptr<TonavBodyStateOdometry> odometry(new TonavBodyStateOdometry(sim_setup));
    
    return odometry;
}

void TonavBodyStateOdometry::initialize(VioSimulation *simulation) {
    vio_simulation_ = simulation;
    tonav_calibration_ = TonavCalibration::prepare(sim_setup_, "tonav_params.json");
    const Imu& imu = sim_setup_->getImu();
    const Trajectory &trajectory = sim_setup_->getTrajectory();
    next_propagation_time_ = NAN;
    double time = 0;
    Eigen::Vector3d rotation_estimate = imu.getGyroscopeData(time);
    Eigen::Vector3d acceleration_estimate = imu.getAccelerometerData(time);
    tonav::Quaternion q_B_G = trajectory.getGlobalToBodyFrameRotation(time);
    Eigen::Vector3d p_B_G = trajectory.getBodyPositionInGlobalFrame(time);
    Eigen::Vector3d v_B_G = imu.getVelocity(time);
    tonav_body_state_ = std::make_shared<tonav::BodyState>(tonav_calibration_, time, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
}

void TonavBodyStateOdometry::updateAcceleration(double time, const Eigen::Vector3d& accel) {
    bool next_propagation_time_was_nan = std::isnan(next_propagation_time_);
    if (next_propagation_time_was_nan) {
        next_propagation_time_ = time;
    }
    if (std::abs(time - next_propagation_time_) >= 1e-6) {
        throw std::runtime_error("Bad IMU timing.");
    }
    
    next_accel_ = accel;
    if (!next_propagation_time_was_nan) {
        propagateBodyState();
    }
}

void TonavBodyStateOdometry::updateRotationRate(double time, const Eigen::Vector3d& gyro) {
    bool next_propagation_time_was_nan = std::isnan(next_propagation_time_);
    if (next_propagation_time_was_nan) {
        next_propagation_time_ = time;
    }
    if (std::abs(time - next_propagation_time_) >= 1e-6) {
        throw std::runtime_error("Bad IMU timing.");
    }
    
    next_gyro_ = gyro;
    if (!next_propagation_time_was_nan) {
        propagateBodyState();
    }
}

void TonavBodyStateOdometry::updateFrame(double time, const cv::Mat& frame) {
    // Do nothing.
}

Eigen::Vector3d TonavBodyStateOdometry::getBodyPositionInGlobalFrame() const {
    return tonav_body_state_->getPositionInGlobalFrame();
}

tonav::Quaternion TonavBodyStateOdometry::getGlobalToBodyFrameRotation() const {
    return tonav_body_state_->getOrientationInGlobalFrame();
}

Eigen::Vector3d TonavBodyStateOdometry::getCameraPositionInGlobalFrame() const {
    return getBodyPositionInGlobalFrame();
}

tonav::Quaternion TonavBodyStateOdometry::getGlobalToCameraFrameRotation() const {
    return getGlobalToBodyFrameRotation();
}

TonavBodyStateOdometry::~TonavBodyStateOdometry() = default;

TonavBodyStateOdometry::TonavBodyStateOdometry(SimSetup *sim_setup) : Odometry(sim_setup) { }

void TonavBodyStateOdometry::propagateBodyState() {
    std::shared_ptr<tonav::BodyState> next_body_state = tonav::BodyState::propagate(*tonav_body_state_, next_propagation_time_, next_gyro_, next_accel_);
    tonav_body_state_ = next_body_state;
    next_propagation_time_ = NAN;
}
