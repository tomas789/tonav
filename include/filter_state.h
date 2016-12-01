//
// Created by Tomas Krejci on 5/17/16.
//

#ifndef TONAV_FILTER_STATE_H
#define TONAV_FILTER_STATE_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>

#include "body_state.h"
#include "calibration.h"
#include "camera_pose.h"
#include "camera_pose_buffer.h"

class Filter;

/**
 * @brief Filter state \f$ \mathbf{x}_k \f$.
 */
class FilterState {
public:
    friend class Filter;
    friend std::ostream& operator<<(std::ostream&, FilterState&);

    FilterState(std::shared_ptr<const Calibration> caliration);
    
    double time() const;

    const Eigen::Quaterniond& getOrientationInGlobalFrame() const;
    const Eigen::Vector3d& getPositionInGlobalFrame() const;
    const Eigen::Vector3d& getVelocityInGlobalFrame() const;

    CameraPoseBuffer& poses();
    const CameraPoseBuffer& poses() const;
private:
    /**
     * Calibration is used to:
     *  1/ determine maximum number of camera poses stored in filter state
     *  2/ initialize first `BodyState` instance using
     */
    std::shared_ptr<const Calibration> calibration_;

    /** @brief Contains body pose */
    std::shared_ptr<BodyState> body_state_;

    Eigen::Vector3d bias_accelerometer_;

    Eigen::Vector3d bias_gyroscope_;

    /** @brief \f$T_g\f$ */
    Eigen::Matrix3d gyroscope_shape_;
    
    /** @brief \f$T_s\f$ */
    Eigen::Matrix3d gyroscope_acceleration_sensitivity_;
    
    /** @brief \f$T_a\f$ */
    Eigen::Matrix3d accelerometer_shape_;

    Eigen::Vector3d position_of_body_in_camera_;

    Eigen::Vector2d focal_point_;

    Eigen::Vector2d optical_center_;

    Eigen::Vector3d radial_distortion_;

    Eigen::Vector2d tangential_distortion_;

    double camera_delay_;

    double camera_readout_;
    
    /** @brief All camera poses */
    CameraPoseBuffer poses_;

};

std::ostream& operator<< (std::ostream& out, FilterState& state);

#endif //TONAV_FILTER_STATE_H
