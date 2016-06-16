#include "body_state.h"

BodyState::BodyState() {
    body_state_.setZero();
    setRotationQuaternion(Eigen::Quaterniond::Identity());
}

Eigen::Block<BodyState::BodyStateType, 4, 1> BodyState::getRotationBlock() {
    return body_state_.block<4, 1>(0, 0);
}

const Eigen::Block<const BodyState::BodyStateType, 4, 1> BodyState::getRotationBlock() const {
    return body_state_.block<4, 1>(0, 0);
}

Eigen::Quaterniond BodyState::getRotationQuaternion() const {
    const Eigen::Block<const BodyStateType, 4, 1> rotation_block = getRotationBlock();
    Eigen::Quaterniond rot(rotation_block(3, 0), rotation_block(0, 0), rotation_block(1, 0), rotation_block(2, 0));
    return rot;
}

void BodyState::setRotationQuaternion(const Eigen::Quaterniond &quat) {
    Eigen::Block<BodyStateType, 4, 1> rotation_block = getRotationBlock();
    rotation_block(0, 0) = quat.x();
    rotation_block(1, 0) = quat.y();
    rotation_block(2, 0) = quat.z();
    rotation_block(3, 0) = quat.w();
}

Eigen::Block<BodyState::BodyStateType, 3, 1> BodyState::getPositionBlock() {
    return body_state_.block<3, 1>(4, 0);
}

const Eigen::Block<const BodyState::BodyStateType, 3, 1> BodyState::getPositionBlock() const {
    return body_state_.block<3, 1>(4, 0);
}

Eigen::Block<BodyState::BodyStateType, 3, 1> BodyState::getVelocityBlock() {
    return body_state_.block<3, 1>(7, 0);
}

const Eigen::Block<const BodyState::BodyStateType, 3, 1> BodyState::getVelocityBlock() const {
    return body_state_.block<3, 1>(7, 0);
}

Eigen::Block<BodyState::BodyStateType, 3, 1> BodyState::getAccelerometerBiasBlock() {
    return body_state_.block<3, 1>(10, 0);
}

const Eigen::Block<const BodyState::BodyStateType, 3, 1> BodyState::getAccelerometerBiasBlock() const {
    return body_state_.block<3, 1>(10, 0);
}

Eigen::Block<BodyState::BodyStateType, 3, 1> BodyState::getGyroscopeBiasBlock() {
    return body_state_.block<3, 1>(13, 0);
}

const Eigen::Block<const BodyState::BodyStateType, 3, 1> BodyState::getGyroscopeBiasBlock() const {
    return body_state_.block<3, 1>(13, 0);
}

Eigen::Vector3d& BodyState::getRotationEstimateRef() {
    return rotation_estimate_;
}

const Eigen::Vector3d& BodyState::getRotationEstimateRef() const {
    return rotation_estimate_;
}

Eigen::Vector3d& BodyState::getAccelerationEstimateRef() {
    return acceleration_estimate_;
}

const Eigen::Vector3d& BodyState::getAccelerationEstimateRef() const {
    return acceleration_estimate_;
}

Eigen::Quaterniond& BodyState::getRotationToThisFrameRef() {
    return rotation_to_this_frame_;
}

const Eigen::Quaterniond& BodyState::getRotationToThisFrameRef() const {
    return rotation_to_this_frame_;
}