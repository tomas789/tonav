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
    
    gyroscope_shape_ += delta_T_g;
    gyroscope_acceleration_sensitivity_ += delta_T_s;
    accelerometer_shape_ += delta_T_a;
    
//    position_of_body_in_camera_ += delta_x.segment<3>(42);
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
    out << "Gyro shape:      " << std::endl << state.gyroscope_shape_.format(formatter) << std::endl;
    out << "G-Sensitivity:   " << std::endl << state.gyroscope_acceleration_sensitivity_.format(formatter) << std::endl;
    out << "Accel shape:     " << std::endl << state.accelerometer_shape_.format(formatter)
        << std::endl;
    out << "p_B_C:           " << state.position_of_body_in_camera_.transpose().format(formatter) << std::endl;
//    out << "Focal length:    " << state.getFocalLengthXRef() << ", " << state.getFocalLengthYRef() << std::endl;
//    out << "Optical center:  " << state.getOpticalCenterXRef() << ", " << state.getOpticalCenterYRef() << std::endl;
    out << "Radial dist:     " << state.radial_distortion_.transpose().format(formatter) << std::endl;
    out << "Tangential dist: " << state.tangential_distortion_.transpose().format(formatter) << std::endl;
    out << "Cam delay:       " << state.camera_delay_ << std::endl;
    out << "Cam readout:     " << state.camera_readout_ << std::endl;

//    for (int i = 0; i < FilterState::getPoses(); ++i) {
//        out << " * " << std::setw(8) << "Pose " + std::to_string(i) << ": "
//            << state.getRotationForBodyPoseBlock(i).transpose().format(formatter) << " "
//            << state.getPositionForBodyPoseBlock(i).transpose().format(formatter) << " "
//            << state.getVelocityForBodyPoseBlock(i).transpose().format(formatter) << std::endl;
//    }

    return out;
}
