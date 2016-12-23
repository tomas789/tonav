#include "state_initializer.h"

#include <Eigen/Core>

#include "quaternion.h"

StateInitializer::StateInitializer()
        : orientation_(Quaternion::identity()) {
    position_ = Eigen::Vector3d::Zero();
    velocity_ = Eigen::Vector3d::Zero();
}

void StateInitializer::setOrientation(const Quaternion& orientation) {
    orientation_ = orientation;
}

Quaternion StateInitializer::getOrientation() const {
    return orientation_;
}

void StateInitializer::setPosition(const Eigen::Vector3d& position) {
    position_ = position;
}

Eigen::Vector3d StateInitializer::getPosition() const {
    return position_;
}

void StateInitializer::setVelocity(const Eigen::Vector3d& velocity) {
    velocity_ = velocity;
}

Eigen::Vector3d StateInitializer::getVelocity() const {
    return velocity_;
}
