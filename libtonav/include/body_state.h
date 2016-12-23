#ifndef TONAV_BODY_STATE_H
#define TONAV_BODY_STATE_H

#include <Eigen/Core>

#include "calibration.h"
#include "quaternion.h"

class Filter;

/**
 * @brief Represents body pose \f$x_B\f$ or camera pose \f$ \pi_{B_i} \f$ in filter state.
 */
class BodyState {
public:
    /**
     * @brief Create new body state using estimates
     *
     * @param time Time for which rotation and acceleration estimates are valid.
     * @param rotation_estimate Estimate of angular velocity \f$ \prescript{B}{}{\boldsymbol{\hat{\omega}}}(t) \f$.
     *                          Already cleared for distortions.
     * @param acceleration_estimate Estimate of linear acceleration \f$ \prescript{B}{}{\mathbf{\hat{a}}(t) \f$.
     *                              Already cleared for distortions.
     * @param q_B_G Orientation of body frame w.r.t global frame \f$ \prescript{G}{}{\mathbf{q}}_B \f$
     * @param p_B_G Position of body frame in global frame \f$ \prescript{G}{}{\mathbf{p}}_B \f$
     * @param v_B_G Velocity of body frame in global frame \f$ \prescript{G}{}{\mathbf{v}}_B \f$
     */
    BodyState(std::shared_ptr<const Calibration> calibration, double time, Eigen::Vector3d rotation_estimate,
            Eigen::Vector3d acceleration_estimate, Quaternion q_B_G, Eigen::Vector3d p_B_G,
            Eigen::Vector3d v_B_G);

    BodyState(const BodyState& other) = default;
    BodyState(BodyState&& other) = default;

    BodyState& operator=(const BodyState& other) = default;
    BodyState& operator=(BodyState&& other) = default;

    /**
     * @brief Create new state propagated to next time period.
     *
     * @param time Time for which rotation and acceleration estimates are valid.
     * @param rotation_estimate Estimate of angular velocity \f$ \prescript{B}{}{\boldsymbol{\hat{\omega}}}(t) \f$.
     *                          Already cleared for distortions.
     * @param acceleration_estimate Estimate of linear acceleration \f$ \prescript{B}{}{\mathbf{\hat{a}}(t) \f$.
     *                              Already cleared for distortions.
     * @return New, propagated body state.
     */
    static std::shared_ptr<BodyState> propagate(
            const BodyState& from_state, double time, Eigen::Vector3d rotation_estimate,
            Eigen::Vector3d acceleration_estimate);

    static Eigen::Matrix<double, 56, 56> propagateCovariance(const Filter& filter, const BodyState& from_state,
            BodyState& to_state, const Eigen::Matrix<double, 56, 56>& covar);

    /**
     * @brief Time of body state
     */
    double time() const;

    /** @brief Get time delta from this to `to_state` state */
    double timeTo(const BodyState& to_state) const;

    /** @brief Get orientation \f$ \prescript{G}{}{\mathbf{q}}_B \f$ of this body (or camera) frame in global frame. */
    const Quaternion& getOrientationInGlobalFrame() const;

    /** @brief Get position \f$ \prescript{G}{}{\mathbf{p}}_B \f$ of this body (or camera) frame in global frame. */
    const Eigen::Vector3d& getPositionInGlobalFrame() const;

    /** @brief Get velocity \f$ \prescript{G}{}{\mathbf{v}}_B \f$ of this body (or camera) frame in global frame. */
    const Eigen::Vector3d& getVelocityInGlobalFrame() const;
    
    void orientationCorrection(const Quaternion& orientation);
    void positionCorrection(const Eigen::Vector3d& position);
    void velocityCorrection(const Eigen::Vector3d& velocity);
    
    void updateWithStateDelta(const Eigen::VectorXd& delta_x);
private:
    std::shared_ptr<const Calibration> calibration_;

    /** @brief Time at which current BodyState was created */
    double time_;

    /** @brief \f$ \prescript{G}{}{\mathbf{q}}_B = \prescript{B}{G}{\mathbf{q}} \f$ */
    Quaternion q_B_G_;

    /** @brief \f$ \prescript{G}{}{\mathbf{p}}_B \f$ */
    Eigen::Vector3d p_B_G_;

    /** @brief \f$ \prescript{G}{}{\mathbf{v}}_B \f$ */
    Eigen::Vector3d v_B_G_;

    /** @brief \f$ \prescript{B}{}{\boldsymbol{\hat{\omega}}}(t) \f$ */
    Eigen::Vector3d rotation_estimate_;

    /** @brief \f$ \prescript{B}{}{\mathbf{\hat{a}}}(t) \f$ */
    Eigen::Vector3d acceleration_estimate_;

    /** @brief \f$ \prescript{B_t}{B_{t-1}}{\hat{q}} \f$ */
    Quaternion rotation_to_this_frame_;

    /** @brief As in Shelley (6.29) */
    Eigen::Vector3d s_hat_ = Eigen::Vector3d::Zero();

    /** @brief As in Shelley (6.30) */
    Eigen::Vector3d y_hat_ = Eigen::Vector3d::Zero();

    /**
     * @brief Integrate local angular velocity
     *
     * Should return rotation from \f$\{B_{l+1}\}\f$ to \f$\{B_{l}\}\f$
     */
    static Quaternion propagateGyroscope(const BodyState& from_state, BodyState& to_state);

    static std::pair<Eigen::Vector3d, Eigen::Vector3d> propagateAccelerometer(
            const BodyState& from_state, BodyState& to_state, const Quaternion& q_Bnext_Bcurrent);

    static Eigen::Matrix<double, 15, 15> bodyStateTransitionMatrix(const Filter& filter, const BodyState& from_state,
            BodyState& to_state);

    static Eigen::Matrix<double, 15, 27> imuCalibrationParamsTransitionMatrix(const Filter& filter,
            const BodyState& from_state, BodyState& to_state);

    static Eigen::Matrix<double, 15, 15> propagationNoiseMatrix(const Filter& filter, const BodyState& from_state,
            BodyState& to_state, const Eigen::Matrix<double, 15, 15>& bodyStateTransitionMatrix);
};

#endif //TONAV_BODY_STATE_H
