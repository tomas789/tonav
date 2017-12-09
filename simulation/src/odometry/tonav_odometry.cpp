//
// Created by Tomas Krejci on 10/28/17.
//

#include "odometry/tonav_odometry.h"

#include <tonav.h>
#include <geometry.h>

#include "sim_setup_component.h"
#include "vio_simulation.h"

std::unique_ptr<TonavOdometry> TonavOdometry::load(SimSetup* sim_setup, const json& j) {
    std::unique_ptr<TonavOdometry> odometry(new TonavOdometry(sim_setup));
    
    odometry->tonav_calibration_ = TonavCalibration::prepare(sim_setup, "/Users/tomaskrejci/Developer/tonav/examples/tonav_params_0.json");
    
    return odometry;
}

void TonavOdometry::initialize(VioSimulation *simulation) {
    vio_simulation_ = simulation;
    Vision &vision = sim_setup_->getVision();
    next_propagation_time_ = NAN;
    
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
    bool was_updated = false;
    updateAcceleration(time, accel, was_updated);
}

void TonavOdometry::updateAcceleration(double time, const Eigen::Vector3d& accel, bool& was_updated) {
    if (!tonav_->isInitialized()) {
        updateTonavInitializerFromGroundTruth(time);
    }
    was_updated = tonav_->updateAcceleration(time, accel);
    if (was_updated) {
        evaluateToGroundTruth(time);
    }
}

void TonavOdometry::updateRotationRate(double time, const Eigen::Vector3d& gyro) {
    bool was_updated = false;
    updateRotationRate(time, gyro, was_updated);
}

void TonavOdometry::updateRotationRate(double time, const Eigen::Vector3d& gyro, bool& was_updated) {
    if (!tonav_->isInitialized()) {
        updateTonavInitializerFromGroundTruth(time);
    }
    was_updated = tonav_->updateRotationRate(time, gyro);
    if (was_updated) {
        evaluateToGroundTruth(time);
    }
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
    if (!tonav_->isInitialized()) {
        return Eigen::Vector3d::Zero();
    }
    tonav::Quaternion q_C_B = tonav_->filter().getBodyToCameraRotation();
    Eigen::Vector3d p_B_C = tonav_->filter().getPositionOfBodyInCameraFrame();
    Eigen::Vector3d p_B_G = getBodyPositionInGlobalFrame();
    tonav::Quaternion q_B_G = getGlobalToBodyFrameRotation();
    tonav::Quaternion q_G_B = q_B_G.conjugate();
    Eigen::Vector3d p_G_B = tonav::Geometry::switchFrames(p_B_G, q_B_G);
    tonav::Quaternion q_B_C = q_C_B.conjugate();
    Eigen::Vector3d p_C_B = tonav::Geometry::switchFrames(p_B_C, q_B_C);
    Eigen::Vector3d p_C_G = tonav::Geometry::transformFrames(p_C_B, q_G_B, p_G_B);
    return p_C_G;
}

tonav::Quaternion TonavOdometry::getGlobalToCameraFrameRotation() const {
    if (!tonav_->isInitialized()) {
        return tonav::Quaternion::identity();
    }
    tonav::Quaternion q_B_G = getGlobalToBodyFrameRotation();
    tonav::Quaternion q_C_B = tonav_calibration_->getBodyToCameraRotation();
    tonav::Quaternion q_C_G = q_C_B*q_B_G;
    return q_C_G;
}

TonavOdometry::~TonavOdometry() {
    std::ofstream out("gt_eval.json");
    out << eval_to_gt_.dump(4);
}

std::shared_ptr<tonav::Tonav> TonavOdometry::getTonav() {
    return tonav_;
}

std::shared_ptr<TonavCalibration> TonavOdometry::getTonavCalibration() {
    return tonav_calibration_;
}

TonavOdometry::TonavOdometry(SimSetup *sim_setup) : Odometry(sim_setup) { }

void TonavOdometry::updateTonavInitializerFromGroundTruth(double time) {
    const Trajectory& trajectory = sim_setup_->getTrajectory();
    const Imu& imu = sim_setup_->getImu();
    std::shared_ptr<tonav::StateInitializer> initializer = tonav_->initializer();
    initializer->setOrientation(trajectory.getGlobalToBodyFrameRotation(time));
    initializer->setPosition(trajectory.getBodyPositionInGlobalFrame(time));
    initializer->setVelocity(imu.getVelocity(time));
}

void TonavOdometry::evaluateToGroundTruth(double time) {
    const Trajectory& trajectory = sim_setup_->getTrajectory();
    const Imu& imu = sim_setup_->getImu();
    
    const tonav::FilterState& state = tonav_->state();
    
    const auto& q_B_G = state.getOrientationInGlobalFrame();
    const auto& p_B_G = state.getPositionInGlobalFrame();
    const auto& v_B_G = state.getVelocityInGlobalFrame();
    const auto& b_g = state.getGyroscopeBias();
    const auto& b_a = state.getAccelerometerBias();
    const auto& p_B_C = state.getBodyPositionInCameraFrame();
    const auto& focal_length = state.getFocalLength();
    const auto& optical_center = state.getOpticalCenter();
    const auto& radial_distortion = state.getRadialDistortion();
    const auto& tangential_distortion = state.getTangentialDistortion();
    double camera_delay = state.getCameraDelayTime();
    double camera_readout = state.getCameraReadoutTime();
    
    const auto& q_B_G_gt = trajectory.getGlobalToBodyFrameRotation(state.time());
    const auto& p_B_G_gt = trajectory.getBodyPositionInGlobalFrame(state.time());
    const auto& v_B_G_gt = imu.getVelocity(state.time());
    Eigen::Vector3d b_g_gt = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_a_gt = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_B_C_gt = tonav::Geometry::switchFrames(trajectory.getCameraPositionInBodyFrame(), tonav_calibration_->getBodyToCameraRotation());
    Eigen::Vector2d focal_length_gt = tonav_calibration_->getCameraFocalLength();
    Eigen::Vector2d optical_center_gt = tonav_calibration_->getCameraOpticalCenter();
    Eigen::Vector3d radial_distortion_gt = tonav_calibration_->getCameraRadialDistortionParams();
    Eigen::Vector2d tangential_distortion_gt = tonav_calibration_->getCameraTangentialDistortionParams();
    double camera_delay_gt = tonav_calibration_->getCameraDelayTime();
    double camera_readout_gt = tonav_calibration_->getCameraReadoutTime();
    
    json j;
    
    writeEvalGt(j, "q_B_G", q_B_G_gt, q_B_G);
    writeEvalGt(j, "p_B_G", p_B_G_gt, p_B_G);
    writeEvalGt(j, "v_B_G", v_B_G_gt, v_B_G);
    writeEvalGt(j, "b_a", b_a_gt, b_a);
    writeEvalGt(j, "b_g", b_g_gt, b_g);
    writeEvalGt(j, "p_B_C", p_B_C_gt, p_B_C);
    writeEvalGt(j, "focal_length", focal_length_gt, focal_length);
    writeEvalGt(j, "optical_center", optical_center_gt, optical_center);
    writeEvalGt(j, "camera_delay", camera_delay_gt, camera_delay);
    writeEvalGt(j, "camera_readout", camera_readout_gt, camera_readout);
    
    eval_to_gt_[std::to_string(state.time())] = j;
}

void TonavOdometry::writeEvalGt(json& j, const std::string& key, const Eigen::Vector2d& true_state, const Eigen::Vector2d& estimate) {
    Eigen::Vector2d err = true_state - estimate;
    j[key] = json {
        { "true", true_state },
        { "estimate", estimate },
        { "error", err },
        { "error_norm", err.norm() }
    };
}
void TonavOdometry::writeEvalGt(json& j, const std::string& key, const Eigen::Vector3d& true_state, const Eigen::Vector3d& estimate) {
    Eigen::Vector3d err = true_state - estimate;
    j[key] = json {
        { "true", true_state },
        { "estimate", estimate },
        { "error", err },
        { "error_norm", err.norm() }
    };
}
void TonavOdometry::writeEvalGt(json& j, const std::string& key, const tonav::Quaternion& true_state, const tonav::Quaternion& estimate) {
    tonav::Quaternion err = true_state.conjugate()*estimate;
    j[key] = json {
        { "true", true_state },
        { "estimate", estimate },
        { "error", err },
        { "error_norm", err.angularDistance(tonav::Quaternion::identity())}
    };
    
}
void TonavOdometry::writeEvalGt(json& j, const std::string& key, double true_state, double estimate) {
    double err = true_state - estimate;
    j[key] = json {
        { "true", true_state },
        { "estimate", estimate },
        { "error", err },
        { "error_norm", err }
    };
}

