#include "state_initializer.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

StateInitializer::StateInitializer() {
    orientation_ = Eigen::Quaterniond::Identity();
    position_ = Eigen::Vector3d::Zero();
    velocity_ = Eigen::Vector3d::Zero();
}

void StateInitializer::setOrientation(const Eigen::Quaterniond& orientation) {
    orientation_ = orientation;
}

Eigen::Quaterniond StateInitializer::getOrientation() const {
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
