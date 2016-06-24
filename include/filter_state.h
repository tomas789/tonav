//
// Created by Tomas Krejci on 5/17/16.
//

#ifndef TONAV_FILTER_STATE_H
#define TONAV_FILTER_STATE_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include "body_state.h"
#include "camera_pose.h"
#include "ring_buffer.h"

class FilterState {
public:
    using StateType = Eigen::Matrix<double, 57, 1>;
    
    FilterState(int max_camera_poses);
    FilterState(const FilterState& other) = default;
    FilterState(FilterState&& other) = default;
    
    FilterState& operator= (const FilterState& other) = default;
    FilterState& operator= (FilterState&& other) = default;
    
    BodyState& getBodyStateRef();
    const BodyState& getBodyStateRef() const;

    Eigen::Block<BodyState::BodyStateType, 4, 1> getRotationBlock();
    Eigen::Quaterniond getRotationQuaternion();
    void setRotationQuaternion(const Eigen::Quaterniond& quat);
    Eigen::Block<BodyState::BodyStateType, 3, 1> getPositionBlock();
    Eigen::Block<BodyState::BodyStateType, 3, 1> getVelocityBlock();
    Eigen::Block<BodyState::BodyStateType, 3, 1> getAccelerometerBiasBlock();
    Eigen::Block<BodyState::BodyStateType, 3, 1> getGyroscopeBiasBlock();

    Eigen::Block<StateType, 9, 1> getGyroscopeShapeVectorizedBlock();
    Eigen::Block<StateType, 9, 1> getGSensitivityVectorizedBlock();
    Eigen::Block<StateType, 9, 1> getAccelerometerShapeVectorizedBlock();

    Eigen::Block<StateType, 3, 1> getCameraToBodyOffsetBlock();

    double& getFocalLengthXRef();
    double& getFocalLengthYRef();
    double& getOpticalCenterXRef();
    double& getOpticalCenterYRef();

    Eigen::Block<StateType, 3, 1> getRadialDistortionParametersBlock();
    Eigen::Block<StateType, 2, 1> getTangentialDistortionParametersBlock();

    double& getCameraDelayTimeRef();
    double& getCameraReadoutTimeRef();

    Eigen::Block<Eigen::Vector3d, 3, 1> getRotationEstimateBlock();
    Eigen::Block<Eigen::Vector3d, 3, 1> getAccelerationEstimateBlock();

    Eigen::Quaterniond getRotationToThisFrame();
    void setRotationToThisFrame(const Eigen::Quaterniond& quat);

    std::ostream& uglyPrint(std::ostream& out) const;

    FilterState deriveNewStateForImuPropagation() const;

    RingBuffer<CameraPose>& poses();
private:
    /** @brief Contains body pose */
    BodyState body_state_;
    
    /** @brief \f$T_g\f$ */
    Eigen::Matrix3d gyroscope_shape_;
    
    /** @brief \f$T_s\f$ */
    Eigen::Matrix3d g_sensitivity_;
    
    /** @brief \f$T_a\f$ */
    Eigen::Matrix3d accelerometer_shape_;
    
    /** @brief All camera poses */
    RingBuffer<CameraPose> poses_;
    
    /**
     * @birief Filter state except body pose and camera poses
     *
     * @deprecated
     */
    StateType state_;
    
    Eigen::Quaterniond rotation_to_this_frame_;
    Eigen::Vector3d rotation_estimate_;
    Eigen::Vector3d acceleration_estimate_;

    
};

std::ostream& operator<< (std::ostream& out, FilterState& state);

#endif //TONAV_FILTER_STATE_H
