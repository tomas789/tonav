//
// Created by Tomas Krejci on 6/6/16.
//

#ifndef TONAV_CAMERA_POSE_H
#define TONAV_CAMERA_POSE_H

#include <Eigen/Dense>
#include <limits>
#include <set>

class CameraPose {
public:
    using CameraPoseType = Eigen::Matrix<double, 10, 1>;

    CameraPose();
    CameraPose(std::size_t pose_id);

    std::size_t poseId() const;

    std::size_t getActiveFeaturesCount() const;
    void setActiveFeaturesCount(std::size_t i);
    void decreaseActiveFeaturesCount(int feature_id);

    Eigen::Block<CameraPoseType, 4, 1> getRotationForBodyPoseBlock();
    Eigen::Block<CameraPoseType, 3, 1> getPositionForBodyPoseBlock();
    Eigen::Block<CameraPoseType, 3, 1> getVelocityForBodyPoseBlock();

    bool isValid() const;
    void rememberFeatureId(int feature_id);

private:
    std::set<int> feature_ids_;
    CameraPoseType camera_pose_;
    std::size_t pose_id_ = std::numeric_limits<std::size_t>::max();
    std::size_t features_active_;
};

#endif //TONAV_CAMERA_POSE_H
