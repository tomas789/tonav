//
// Created by Tomas Krejci on 5/17/16.
//

#include "filter_state.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "body_state.h"
#include "calibration.h"
#include "quaternion.h"
#include "filter.h"

FilterState::FilterState(std::shared_ptr<const Calibration> calibration)
    : calibration_(calibration), poses_(calibration->getMaxCameraPoses() + 1) { }

double FilterState::time() const {
    return body_state_->time();
}

const Quaternion& FilterState::getOrientationInGlobalFrame() const {
    assert(body_state_.get());
    return body_state_->getOrientationInGlobalFrame();
}

const Eigen::Vector3d& FilterState::getPositionInGlobalFrame() const {
    assert(body_state_.get());
    return body_state_->getPositionInGlobalFrame();
}

const Eigen::Vector3d& FilterState::getVelocityInGlobalFrame() const {
    assert(body_state_.get());
    return body_state_->getVelocityInGlobalFrame();
}

const Eigen::Matrix3d& FilterState::getGyroscopeShapeMatrix() const {
    return gyroscope_shape_;
}

const Eigen::Matrix3d& FilterState::getGyroscopeAccelerationSensitivityMatrix() const {
    return gyroscope_acceleration_sensitivity_;
}

const Eigen::Matrix3d& FilterState::getAccelerometerShapeMatrix() const {
    return accelerometer_shape_;
}

void FilterState::orientationCorrection(const Quaternion& orientation) {
    body_state_->orientationCorrection(orientation);
}

void FilterState::positionCorrection(const Eigen::Vector3d& position) {
    body_state_->positionCorrection(position);
}

void FilterState::velocityCorrection(const Eigen::Vector3d& velocity) {
    body_state_->velocityCorrection(velocity);
}

CameraPoseBuffer& FilterState::poses() {
    return poses_;
}

const CameraPoseBuffer& FilterState::poses() const {
    return poses_;
}

void FilterState::updateWithStateDelta(const Eigen::VectorXd& delta_x) {
    assert(!std::isnan(delta_x.maxCoeff()));
    
    body_state_->updateWithStateDelta(delta_x.topRows(9));
    bias_gyroscope_ += delta_x.segment<3>(9);
    bias_accelerometer_ += delta_x.segment<3>(12);
    
    Eigen::Matrix3d delta_T_g;
    delta_T_g.block<1, 3>(0, 0) = delta_x.segment<3>(15);
    delta_T_g.block<1, 3>(1, 0) = delta_x.segment<3>(18);
    delta_T_g.block<1, 3>(2, 0) = delta_x.segment<3>(21);
    Eigen::Matrix3d delta_T_s;
    delta_T_s.block<1, 3>(0, 0) = delta_x.segment<3>(24);
    delta_T_s.block<1, 3>(1, 0) = delta_x.segment<3>(27);
    delta_T_s.block<1, 3>(2, 0) = delta_x.segment<3>(30);
    Eigen::Matrix3d delta_T_a;
    delta_T_a.block<1, 3>(0, 0) = delta_x.segment<3>(33);
    delta_T_a.block<1, 3>(1, 0) = delta_x.segment<3>(36);
    delta_T_a.block<1, 3>(2, 0) = delta_x.segment<3>(39);
    
    // @todo: Shape matrices?
//    gyroscope_shape_ += delta_T_g;
//    gyroscope_acceleration_sensitivity_ += delta_T_s;
//    accelerometer_shape_ += delta_T_a;
    
    position_of_body_in_camera_ += delta_x.segment<3>(42);
    focal_point_ += delta_x.segment<2>(45);
    optical_center_ += delta_x.segment<2>(47);
    radial_distortion_ += delta_x.segment<3>(49);
    tangential_distortion_ += delta_x.segment<2>(52);
    
    camera_delay_ += delta_x(54);
    camera_readout_ += delta_x(55);
    
    for (std::size_t j = 0; j < poses().size(); ++j) {
        poses()[j].updateWithStateDelta(delta_x.segment<9>(56+j*9));
    }
}
