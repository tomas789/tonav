//
// Created by Tomas Krejci on 5/17/16.
//

#include "filter_state.h"

#include <iostream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Dense>

FilterState::FilterState(std::shared_ptr<const Calibration> calibration)
    : calibration_(calibration), poses_(calibration->getMaxCameraPoses() + 1) { }

double FilterState::time() const {
    return body_state_->time();
}

const Eigen::Quaterniond& FilterState::getOrientationInGlobalFrame() const {
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

CameraPoseBuffer& FilterState::poses() {
    return poses_;
}

const CameraPoseBuffer& FilterState::poses() const {
    return poses_;
}

std::ostream& operator<< (std::ostream& out, FilterState& state) {
    Eigen::IOFormat formatter(4, 0, ", ", "\n", "[", "]");
    out << std::fixed << std::setprecision(4);
    Eigen::Vector3d euler = state.getOrientationInGlobalFrame().toRotationMatrix().eulerAngles(0, 1, 2);
    out << "Euler angles:    " << euler.transpose().format(formatter) << std::endl;
    out << "Rotation:        " << state.getOrientationInGlobalFrame().coeffs().transpose().format(formatter) << std::endl;
    out << "Position:        " << state.getPositionInGlobalFrame().transpose().format(formatter) << std::endl;
    out << "Velocity:        " << state.getVelocityInGlobalFrame().transpose().format(formatter) << std::endl;
    out << "Gyro bias:       " << state.bias_gyroscope_.transpose().format(formatter) << std::endl;
    out << "Accel bias:      " << state.bias_accelerometer_.transpose().format(formatter) << std::endl;
//    out << "Gyro shape:      " << state.getGyroscopeShapeVectorizedBlock().transpose().format(formatter) << std::endl;
//    out << "G-Sensitivity:   " << state.getGSensitivityVectorizedBlock().transpose().format(formatter) << std::endl;
//    out << "Accel shape:     " << state.getAccelerometerShapeVectorizedBlock().transpose().format(formatter)
//        << std::endl;
    out << "p_B_C:           " << state.position_of_body_in_camera_.transpose().format(formatter) << std::endl;
//    out << "Focal length:    " << state.getFocalLengthXRef() << ", " << state.getFocalLengthYRef() << std::endl;
//    out << "Optical center:  " << state.getOpticalCenterXRef() << ", " << state.getOpticalCenterYRef() << std::endl;
//    out << "Radial dist:     " << state.getRadialDistortionParametersBlock().transpose().format(formatter) << std::endl;
//    out << "Tangential dist: " << state.getTangentialDistortionParametersBlock().transpose().format(formatter)
//        << std::endl;
//    out << "Cam delay:       " << state.getCameraDelayTimeRef() << std::endl;
//    out << "Cam readout:     " << state.getCameraReadoutTimeRef() << std::endl;

//    for (int i = 0; i < FilterState::getPoses(); ++i) {
//        out << " * " << std::setw(8) << "Pose " + std::to_string(i) << ": "
//            << state.getRotationForBodyPoseBlock(i).transpose().format(formatter) << " "
//            << state.getPositionForBodyPoseBlock(i).transpose().format(formatter) << " "
//            << state.getVelocityForBodyPoseBlock(i).transpose().format(formatter) << std::endl;
//    }

    return out;
}
