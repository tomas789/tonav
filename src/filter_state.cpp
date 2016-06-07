//
// Created by Tomas Krejci on 5/17/16.
//

#include "filter_state.h"

#include <iostream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Dense>

Eigen::Block<FilterState::StateType, 4, 1> FilterState::getRotationBlock() {
    return state_.block<4, 1>(0, 0);
}

Eigen::Quaterniond FilterState::getRotationQuaternion() {
    Eigen::Block<StateType, 4, 1> rotation_block = getRotationBlock();
    Eigen::Quaterniond rot(rotation_block(3, 0), rotation_block(0, 0), rotation_block(1, 0), rotation_block(2, 0));
    return rot;
}

void FilterState::setRotationQuaternion(const Eigen::Quaterniond &quat) {
    Eigen::Block<StateType, 4, 1> rotation_block = getRotationBlock();
    rotation_block(0, 0) = quat.x();
    rotation_block(1, 0) = quat.y();
    rotation_block(2, 0) = quat.z();
    rotation_block(3, 0) = quat.w();
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getPositionBlock() {
    return state_.block<3, 1>(4, 0);
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getVelocityBlock() {
    return state_.block<3, 1>(7, 0);
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getAccelerometerBiasBlock() {
    return state_.block<3, 1>(10, 0);
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getGyroscopeBiasBlock() {
    return state_.block<3, 1>(13, 0);
}

Eigen::Block<FilterState::StateType, 9, 1> FilterState::getGyroscopeShapeVectorizedBlock() {
    return state_.block<9, 1>(16, 0);
}

Eigen::Block<FilterState::StateType, 9, 1> FilterState::getGSensitivityVectorizedBlock() {
    return state_.block<9, 1>(25, 0);
}

Eigen::Block<FilterState::StateType, 9, 1> FilterState::getAccelerometerShapeVectorizedBlock() {
    return state_.block<9, 1>(34, 0);
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getCameraToBodyOffsetBlock() {
    return state_.block<3, 1>(43, 0);
}

double& FilterState::getFocalLengthXRef() {
    return state_(46, 0);
}

double& FilterState::getFocalLengthYRef() {
    return state_(47, 0);
}

double& FilterState::getOpticalCenterXRef() {
    return state_(48, 0);
}

double& FilterState::getOpticalCenterYRef() {
    return state_(49, 0);
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getRadialDistortionParametersBlock() {
    return state_.block<3, 1>(50, 0);
}

Eigen::Block<FilterState::StateType, 2, 1> FilterState::getTangentialDistortionParametersBlock() {
    return state_.block<2, 1>(53, 0);
}

double& FilterState::getCameraDelayTimeRef() {
    return state_(55, 0);
}

double& FilterState::getCameraReadoutTimeRef() {
    return state_(56, 0);
}

Eigen::Block<FilterState::StateType, 4, 1> FilterState::getRotationForBodyPoseBlock(std::size_t i) {
    return state_.block<4, 1>(57+10*i, 0);
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getPositionForBodyPoseBlock(std::size_t i) {
    return state_.block<3, 1>(57+10*i+4, 0);
}

Eigen::Block<FilterState::StateType, 3, 1> FilterState::getVelocityForBodyPoseBlock(std::size_t i) {
    return state_.block<3, 1>(57+10*i+7, 0);
}

Eigen::Block<Eigen::Vector3d, 3, 1> FilterState::getRotationEstimateBlock() {
    return rotation_estimate_.block<3, 1>(0, 0);
}

Eigen::Block<Eigen::Vector3d, 3, 1> FilterState::getAccelerationEstimateBlock() {
    return acceleration_estimate_.block<3, 1>(0, 0);
}

Eigen::Quaterniond FilterState::getRotationToThisFrame() {
    return rotation_to_this_frame_;
}

void FilterState::setRotationToThisFrame(const Eigen::Quaterniond &quat) {
    rotation_to_this_frame_ = quat;
}

std::ostream &FilterState::uglyPrint(std::ostream &out) const {
    for (int i = 0; i < state_.rows(); ++i) {
        out << std::setw(4) << i << ": " << std::fixed << std::setw(8) << state_(i, 0) << std::endl;
    }
    return out;
}

FilterState FilterState::deriveNewStateForImuPropagation() const {
    FilterState new_state(*this);
    new_state.getRotationBlock().setZero();
    new_state.getPositionBlock().setZero();
    new_state.getVelocityBlock().setZero();
    new_state.setRotationToThisFrame(Eigen::Quaterniond::Identity());
    new_state.getRotationEstimateBlock().setZero();
    new_state.getAccelerationEstimateBlock().setZero();
    return new_state;
}

FilterState::FilterState(std::size_t max_poses) {
    poses_.reserve(max_poses);
}

std::ostream& operator<< (std::ostream& out, FilterState& state) {
    Eigen::IOFormat formatter(4, 0, ", ", "\n", "[", "]");
    out << std::fixed << std::setprecision(4);
    out << "Rotation:        " << state.getRotationBlock().transpose().format(formatter) << std::endl;
    out << "Position:        " << state.getPositionBlock().transpose().format(formatter) << std::endl;
    out << "Velocity:        " << state.getVelocityBlock().transpose().format(formatter) << std::endl;
    out << "Gyro bias:       " << state.getGyroscopeBiasBlock().transpose().format(formatter) << std::endl;
    out << "Accel bias:      " << state.getAccelerometerBiasBlock().transpose().format(formatter) << std::endl;
    out << "Gyro shape:      " << state.getGyroscopeShapeVectorizedBlock().transpose().format(formatter) << std::endl;
    out << "G-Sensitivity:   " << state.getGSensitivityVectorizedBlock().transpose().format(formatter) << std::endl;
    out << "Accel shape:     " << state.getAccelerometerShapeVectorizedBlock().transpose().format(formatter)
        << std::endl;
    out << "Cam-to-body:     " << state.getCameraToBodyOffsetBlock().transpose().format(formatter) << std::endl;
    out << "Focal length:    " << state.getFocalLengthXRef() << ", " << state.getFocalLengthYRef() << std::endl;
    out << "Optical center:  " << state.getOpticalCenterXRef() << ", " << state.getOpticalCenterYRef() << std::endl;
    out << "Radial dist:     " << state.getRadialDistortionParametersBlock().transpose().format(formatter) << std::endl;
    out << "Tangential dist: " << state.getTangentialDistortionParametersBlock().transpose().format(formatter)
        << std::endl;
    out << "Cam delay:       " << state.getCameraDelayTimeRef() << std::endl;
    out << "Cam readout:     " << state.getCameraReadoutTimeRef() << std::endl;

#if 0
    for (int i = 0; i < FilterState::getPoses(); ++i) {
        out << " * " << std::setw(8) << "Pose " + std::to_string(i) << ": "
            << state.getRotationForBodyPoseBlock(i).transpose().format(formatter) << " "
            << state.getPositionForBodyPoseBlock(i).transpose().format(formatter) << " "
            << state.getVelocityForBodyPoseBlock(i).transpose().format(formatter) << std::endl;
    }
#endif


    return out;
}
