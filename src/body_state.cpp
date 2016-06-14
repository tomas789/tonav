#include "body_state.h"

Eigen::Block<BodyState::BodyStateType, 4, 1> BodyState::getRotationBlock() {
    return body_state_.block<4, 1>(0, 0);
}

Eigen::Quaterniond BodyState::getRotationQuaternion() {
    Eigen::Block<BodyStateType, 4, 1> rotation_block = getRotationBlock();
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

Eigen::Block<BodyState::BodyStateType, 3, 1> BodyState::getVelocityBlock() {
    return body_state_.block<3, 1>(7, 0);
}

Eigen::Block<BodyState::BodyStateType, 3, 1> BodyState::getAccelerometerBiasBlock() {
    return body_state_.block<3, 1>(10, 0);
}

Eigen::Block<BodyState::BodyStateType, 3, 1> BodyState::getGyroscopeBiasBlock() {
    return body_state_.block<3, 1>(13, 0);
}