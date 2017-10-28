//
// Created by Tomas Krejci on 10/28/17.
//

#include "odometry/tonav_odometry.h"

#include <tonav.h>

std::unique_ptr<TonavOdometry> TonavOdometry::load(SimSetup* sim_setup, const json& j) {
    std::unique_ptr<TonavOdometry> odometry(new TonavOdometry(sim_setup));
    
    return std::move(odometry);
}

void TonavOdometry::initialize(VioSimulation *simulation) {
    vio_simulation_ = simulation;
    tonav_calibration_ = TonavCalibration::prepare(sim_setup_);
    const Imu& imu = sim_setup_->getImu();
    const Trajectory &trajectory = sim_setup_->getTrajectory();
    Vision &vision = sim_setup_->getVision();
    next_propagation_time_ = NAN;
    double time = 0;
    Eigen::Vector3d rotation_estimate = imu.getGyroscopeData(time);
    Eigen::Vector3d acceleration_estimate = imu.getAccelerometerData(time);
    tonav::Quaternion q_B_G = trajectory.getGlobalToBodyFrameRotation(time);
    Eigen::Vector3d p_B_G = trajectory.getBodyPositionInGlobalFrame(time);
    Eigen::Vector3d v_B_G = imu.getVelocity(time);
    
    Eigen::Vector3d p_B_C = Eigen::Vector3d::Zero();
    
    cv::Ptr<cv::Feature2D> feature_2d = vision.getFeature2D();
    cv::Ptr<cv::DescriptorMatcher> matcher(new cv::FlannBasedMatcher);
    if (!matcher) {
        throw std::runtime_error("Cannot create descriptor matcher.");
    }
    
    std::shared_ptr<tonav::FeatureTracker> feature_tracker = std::make_shared<tonav::FeatureTracker>(feature_2d, feature_2d, matcher);
    
    tonav_ = std::make_shared<tonav::Tonav>(tonav_calibration_, p_B_C, feature_tracker);
}

void TonavOdometry::updateAcceleration(double time, const Eigen::Vector3d& accel) {
    tonav_->updateAcceleration(time, accel);
}

void TonavOdometry::updateRotationRate(double time, const Eigen::Vector3d& gyro) {
    tonav_->updateRotationRate(time, gyro);
}

void TonavOdometry::updateFrame(double time, const cv::Mat& frame) {
    tonav_->updateImage(time, frame);
}

Eigen::Vector3d TonavOdometry::getBodyPositionInGlobalFrame() const {
    if (!tonav_->isInitialized()) {
        return Eigen::Vector3d::Zero();
    }
    return tonav_->getCurrentPosition();
}

tonav::Quaternion TonavOdometry::getGlobalToBodyFrameRotation() const {
    if (!tonav_->isInitialized()) {
        return tonav::Quaternion::identity();
    }
    return tonav_->getCurrentOrientation();
}

Eigen::Vector3d TonavOdometry::getCameraPositionInGlobalFrame() const {
    return getBodyPositionInGlobalFrame();
}

tonav::Quaternion TonavOdometry::getGlobalToCameraFrameRotation() const {
    return getGlobalToBodyFrameRotation();
}

TonavOdometry::~TonavOdometry() = default;

TonavOdometry::TonavOdometry(SimSetup *sim_setup) : Odometry(sim_setup) { }
