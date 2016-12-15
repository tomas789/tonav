//
// Created by Tomas Krejci on 6/6/16.
//

#include "camera_pose.h"

#include <memory>

#include "body_state.h"
#include "filter.h"
#include "quaternion.h"

std::size_t CameraPose::camera_pose_counter = 0;

CameraPose::CameraPose(const BodyState& body_state, ImuBuffer::iterator hint_gyro, ImuBuffer::iterator hint_accel) {
    body_state_ = std::make_shared<BodyState>(body_state);
    hint_gyro_ = hint_gyro;
    hint_accel_ = hint_accel;
    camera_pose_id_ = CameraPose::camera_pose_counter++;
}

std::size_t CameraPose::getActiveFeaturesCount() const {
    return features_active_;
}

void CameraPose::setActiveFeaturesCount(std::size_t i) {
    features_active_ = i;
}

void CameraPose::decreaseActiveFeaturesCount(int feature_id) {
    assert(feature_ids_.find(feature_id) != std::end(feature_ids_));
    feature_ids_.erase(feature_ids_.find(feature_id));
    
    assert(features_active_ > 0);
    features_active_ -= 1;
}

BodyState& CameraPose::getBodyState() {
    return *body_state_;
}

const BodyState& CameraPose::getBodyState() const {
    return *body_state_;
}

double CameraPose::time() const {
    return body_state_->time();
}

const Quaternion& CameraPose::getBodyOrientationInGlobalFrame() const {
    return body_state_->getOrientationInGlobalFrame();
}

Quaternion CameraPose::getCameraOrientationInGlobalFrame(const Filter& filter) const {
    Quaternion q_C_B = filter.getBodyToCameraRotation();
    Quaternion q_B_G = getBodyOrientationInGlobalFrame();
    return q_C_B * q_B_G;
}

const Eigen::Vector3d& CameraPose::getBodyPositionInGlobalFrame() const {
    return body_state_->getPositionInGlobalFrame();
}

Eigen::Vector3d CameraPose::getCameraPositionInGlobalFrame(const Filter& filter) const {
    Quaternion q_G_C = getCameraOrientationInGlobalFrame(filter).conjugate();
    Eigen::Vector3d p_B_G = body_state_->getPositionInGlobalFrame();
    Eigen::Vector3d p_B_C = filter.getPositionOfBodyInCameraFrame();
    return p_B_G - q_G_C.toRotationMatrix() * p_B_C;
}

const Eigen::Vector3d& CameraPose::getBodyVelocityInGlobalFrame() const {
    return body_state_->getVelocityInGlobalFrame();
}

Quaternion CameraPose::getRotationToOtherPose(const CameraPose& other, const Filter& filter) const {
    Quaternion q_Cto_G = other.getCameraOrientationInGlobalFrame(filter);
    Quaternion q_Cfrom_G = getCameraOrientationInGlobalFrame(filter);
    return q_Cto_G * q_Cfrom_G.conjugate();
}

Eigen::Vector3d CameraPose::getPositionOfAnotherPose(const CameraPose& other, const Filter& filter) const {
    Quaternion q_Cfrom_G = getCameraOrientationInGlobalFrame(filter);
    Eigen::Matrix3d R_Cfrom_G = q_Cfrom_G.toRotationMatrix();
    Eigen::Vector3d p_Cto_G = other.getCameraPositionInGlobalFrame(filter);
    Eigen::Vector3d p_Cfrom_G = getCameraPositionInGlobalFrame(filter);
    return R_Cfrom_G * (p_Cto_G - p_Cfrom_G);
}

void CameraPose::rememberFeatureId(int feature_id) {
    assert(feature_ids_.find(feature_id) == std::end(feature_ids_));
    feature_ids_.insert(feature_id);
}

void CameraPose::updateWithStateDelta(const Eigen::VectorXd& delta_x) {
    body_state_->updateWithStateDelta(delta_x);
}

ImuBuffer::iterator CameraPose::gyroHint() const {
    return hint_gyro_;
}

ImuBuffer::iterator CameraPose::accelHint() const {
    return hint_accel_;
}

std::size_t CameraPose::getCameraPoseId() const {
    return camera_pose_id_;
}








