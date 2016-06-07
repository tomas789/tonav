//
// Created by Tomas Krejci on 6/6/16.
//

#include "camera_pose.h"


CameraPose::CameraPose() {

}

CameraPose::CameraPose(std::size_t pose_id) {
    pose_id_ = pose_id;
}

std::size_t CameraPose::poseId() const {
    return pose_id_;
}

std::size_t CameraPose::getActiveFeaturesCount() const {
    return features_active_;
}

void CameraPose::setActiveFeaturesCount(std::size_t i) {
    features_active_ = i;
}

void CameraPose::decreaseActiveFeaturesCount() {
    features_active_ -= 1;
}

Eigen::Block<CameraPose::CameraPoseType, 4, 1> CameraPose::getRotationForBodyPoseBlock() {
    return camera_pose_.block<4, 1>(0, 0);
}

Eigen::Block<CameraPose::CameraPoseType, 3, 1> CameraPose::getPositionForBodyPoseBlock() {
    return camera_pose_.block<3, 1>(4, 0);
}

Eigen::Block<CameraPose::CameraPoseType, 3, 1> CameraPose::getVelocityForBodyPoseBlock() {
    return camera_pose_.block<3, 1>(7, 0);
}

bool CameraPose::isValid() const {
    return pose_id_ != std::numeric_limits<std::size_t>::max() && features_active_ > 0;
}


















