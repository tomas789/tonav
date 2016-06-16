#ifndef TONAV_BODY_STATE_H
#define TONAV_BODY_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief Represents body pose \f$x_B\f$ in filter state.
 *
 * I considered merging this with class CameraPose which has a lot in common with
 * this class. I decided not to merge them because BodyState has extra \f$b_g\f$ 
 * and \f$b_a\f$.
 */
class BodyState {
public:
    BodyState();
    
    /** @brief Type of data structure that holds parameters of BodyState */
    using BodyStateType = Eigen::Matrix<double, 16, 1>;
    
    /**
     * @brief Get part of BodyState that represents rotation 
     */
    Eigen::Block<BodyStateType, 4, 1> getRotationBlock();
    const Eigen::Block<const BodyStateType, 4, 1> getRotationBlock() const;
    Eigen::Quaterniond getRotationQuaternion() const;
    void setRotationQuaternion(const Eigen::Quaterniond& quat);
    Eigen::Block<BodyStateType, 3, 1> getPositionBlock();
    const Eigen::Block<const BodyStateType, 3, 1> getPositionBlock() const;
    Eigen::Block<BodyStateType, 3, 1> getVelocityBlock();
    const Eigen::Block<const BodyStateType, 3, 1> getVelocityBlock() const;
    
    Eigen::Block<BodyStateType, 3, 1> getAccelerometerBiasBlock();
    const Eigen::Block<const BodyStateType, 3, 1> getAccelerometerBiasBlock() const;
    
    
    Eigen::Block<BodyStateType, 3, 1> getGyroscopeBiasBlock();
    const Eigen::Block<const BodyStateType, 3, 1> getGyroscopeBiasBlock() const;
    
    /** @brief \f$ {}^{B_{l}}\hat{\omega}(t) \f$ */
    Eigen::Vector3d& getRotationEstimateRef();
    const Eigen::Vector3d& getRotationEstimateRef() const;
    
    /** @brief \f$ {}^{B_{l}}\hat{a}(t) \f$ */
    Eigen::Vector3d& getAccelerationEstimateRef();
    const Eigen::Vector3d& getAccelerationEstimateRef() const;
    
    /** @brief \f$ {}_{B_l}^{B_{l+1}}q \f$ */
    Eigen::Quaterniond& getRotationToThisFrameRef();
    const Eigen::Quaterniond& getRotationToThisFrameRef() const;
    
private:
    BodyStateType body_state_;
    
    Eigen::Vector3d rotation_estimate_;
    Eigen::Vector3d acceleration_estimate_;
    Eigen::Quaterniond rotation_to_this_frame_;
};

#endif //TONAV_BODY_STATE_H
