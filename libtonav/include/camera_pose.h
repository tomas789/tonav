//
// Created by Tomas Krejci on 6/6/16.
//

#ifndef TONAV_CAMERA_POSE_H
#define TONAV_CAMERA_POSE_H

#include <Eigen/Core>
#include <limits>
#include <set>
#include <memory>

#include "feature_id.h"
#include "imu_buffer.h"
#include "quaternion.h"

namespace tonav {

class BodyState;

class Filter;

class CameraPose {
public:
    CameraPose(const BodyState &body_state, ImuBuffer::iterator hint_gyro, ImuBuffer::iterator hint_accel, std::size_t frame_id);
    
    std::size_t getActiveFeaturesCount() const;
    
    void setActiveFeaturesCount(std::size_t i);
    
    void decreaseActiveFeaturesCount(const FeatureId& feature_id);
    
    BodyState &getBodyState();
    
    const BodyState &getBodyState() const;
    
    /**
     * @brief Time of body state
     */
    double time() const;
    
    /** @brief Get orientation \f$ {}^G \mathbf{q}_B \f$ of this camera frame in global frame. */
    const Quaternion &getBodyOrientationInGlobalFrame() const;
    
    /** @brief Get orientation \f$ {}^G \mathbf{q}_C \f$ of this camera frame in global frame. */
    Quaternion getCameraOrientationInGlobalFrame(const Filter &filter) const;
    
    /** @brief Get position \f$ {}^G \mathbf{p}_B \f$ of this camera frame in global frame. */
    const Eigen::Vector3d &getBodyPositionInGlobalFrame() const;
    
    /** @brief Get position \f$ {}^G \mathbf{p}_C \f$ of this camera frame in global frame. */
    Eigen::Vector3d getCameraPositionInGlobalFrame(const Filter &filter) const;
    
    /** @brief Get velocity \f$ {}^G\mathbf{v}_B \f$ of this camera frame in global frame. */
    const Eigen::Vector3d &getBodyVelocityInGlobalFrame() const;
    
    Quaternion getRotationToOtherPose(const CameraPose &other, const Filter &filter) const;
    
    Eigen::Vector3d getPositionOfAnotherPose(const CameraPose &other, const Filter &filter) const;
    
    void rememberFeatureId(const FeatureId& feature_id);
    
    void updateWithStateDelta(const Eigen::VectorXd &delta_x);
    
    ImuBuffer::iterator gyroHint() const;
    
    ImuBuffer::iterator accelHint() const;
    
    std::size_t getFrameId() const;
    
    virtual ~CameraPose();

private:
    std::size_t frame_id_;
    std::set<FeatureId> feature_ids_;
    std::size_t features_active_;
    std::shared_ptr<BodyState> body_state_;
    ImuBuffer::iterator hint_gyro_;
    ImuBuffer::iterator hint_accel_;
};

}

#endif //TONAV_CAMERA_POSE_H
