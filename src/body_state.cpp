#include "body_state.h"

#include <Eigen/Core>
#include <memory>
#include <iostream>

#include "calibration.h"
#include "filter.h"
#include "quaternion.h"
#include "quaternion_tools.h"

BodyState::BodyState(std::shared_ptr<const Calibration> calibration, double time, Eigen::Vector3d rotation_estimate,
        Eigen::Vector3d acceleration_estimate, Quaternion q_B_G, Eigen::Vector3d p_B_G, Eigen::Vector3d v_B_G)
: q_B_G_(Quaternion::identity()), rotation_to_this_frame_(Quaternion::identity()) {
    assert(!std::isnan(rotation_estimate.norm()) && rotation_estimate.norm() < 1e6);
    assert(!std::isnan(acceleration_estimate.norm()) && rotation_estimate.norm() < 1e6);
    assert(!std::isnan(q_B_G.norm()));
    assert(!std::isnan(p_B_G.norm()));
    assert(!std::isnan(v_B_G.norm()));
    
    calibration_ = calibration;
    time_ = time;
    q_B_G_ = q_B_G;
    p_B_G_ = p_B_G;
    v_B_G_ = v_B_G;
    rotation_estimate_ = rotation_estimate;
    acceleration_estimate_ = acceleration_estimate;
    rotation_to_this_frame_ = Quaternion::identity();
}

std::shared_ptr<BodyState> BodyState::propagate(const BodyState& from_state, double time,
        Eigen::Vector3d rotation_estimate, Eigen::Vector3d acceleration_estimate) {
    std::shared_ptr<BodyState> to_state = std::make_shared<BodyState>(from_state);
    to_state->time_ = time;
    to_state->rotation_estimate_ = rotation_estimate;
    to_state->acceleration_estimate_ = acceleration_estimate;
    Quaternion q_Bnext_Bcurrent = BodyState::propagateGyroscope(from_state, *to_state);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> p_and_v_delta = BodyState::propagateAccelerometer(
            from_state, *to_state, q_Bnext_Bcurrent);
    Eigen::Vector3d p_delta = p_and_v_delta.first;
    Eigen::Vector3d v_delta = p_and_v_delta.second;

    // Use * instead of *= because quaternion product is not commutative.
    to_state->q_B_G_ = q_Bnext_Bcurrent*from_state.q_B_G_;
    to_state->p_B_G_ = from_state.p_B_G_ + p_delta;
    to_state->v_B_G_ = from_state.v_B_G_ + v_delta;
        
    return to_state;
}

Eigen::Matrix<double, 56, 56> BodyState::propagateCovariance(const Filter& filter, const BodyState& from_state,
        BodyState& to_state, const Eigen::Matrix<double, 56, 56>& covar) {
    // As in Shelley (6.39)

    Eigen::Matrix<double, 15, 15> phi_Bl = BodyState::bodyStateTransitionMatrix(filter, from_state, to_state);
    Eigen::Matrix<double, 15, 27> gamma_imu = BodyState::imuCalibrationParamsTransitionMatrix(filter, from_state, to_state);
    Eigen::Matrix<double, 15, 15> noise_matrix = BodyState::propagationNoiseMatrix(filter, from_state, to_state, phi_Bl);

    Eigen::Matrix<double, 56, 56> phi = Eigen::Matrix<double, 56, 56>::Identity(56, 56);
    phi.block<15, 15>(0, 0) = phi_Bl;
    phi.block<15, 27>(0, 15) = gamma_imu;

    Eigen::Matrix<double, 56, 56> new_covar = phi*covar*phi.transpose();
    new_covar.block<15, 15>(0, 0) += noise_matrix;
    
    return new_covar;
}

double BodyState::time() const {
    return time_;
}

double BodyState::timeTo(const BodyState &to_state) const {
    return to_state.time_ - time_;
}

const Quaternion& BodyState::getOrientationInGlobalFrame() const {
    return q_B_G_;
}

const Eigen::Vector3d& BodyState::getPositionInGlobalFrame() const {
    return p_B_G_;
}

const Eigen::Vector3d& BodyState::getVelocityInGlobalFrame() const {
    return v_B_G_;
}

void BodyState::orientationCorrection(const Quaternion& orientation) {
    q_B_G_ = orientation;
}

void BodyState::positionCorrection(const Eigen::Vector3d& position) {
    p_B_G_ = position;
}

void BodyState::velocityCorrection(const Eigen::Vector3d& velocity) {
    v_B_G_ = velocity;
}

void BodyState::updateWithStateDelta(const Eigen::VectorXd& delta_x) {
    Quaternion delta_q(0.5*delta_x(0), 0.5*delta_x(1), 0.5*delta_x(2), 1.0);
    delta_q.normalize();
    q_B_G_ = (q_B_G_*delta_q).normalized();
    p_B_G_ += delta_x.segment<3>(3);
    v_B_G_ += delta_x.segment<3>(6);
}

Quaternion BodyState::propagateGyroscope(const BodyState &from_state, BodyState &to_state) {
    Eigen::Vector4d q0;
    q0 << 0.0, 0.0, 0.0, 1.0;
    double delta_t = from_state.timeTo(to_state);
    const Eigen::Vector3d& rotation_est_prev = from_state.rotation_estimate_;
    const Eigen::Vector3d& rotation_est = to_state.rotation_estimate_;
    Eigen::Vector3d rotation_est_mid = (rotation_est_prev + rotation_est) / 2.0;

    Eigen::Vector4d k1 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est_prev) * q0;
    Eigen::Vector4d k2 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est_mid) * (q0 + delta_t/2.0 * k1);
    Eigen::Vector4d k3 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est_mid) * (q0 + delta_t/2.0 * k2);
    Eigen::Vector4d k4 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est) * (q0 + delta_t * k3);

    Eigen::Vector4d perturb_vec = q0 + delta_t/6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4);
    Quaternion perturb(perturb_vec(0), perturb_vec(1), perturb_vec(2), perturb_vec(3));
    perturb.normalize();
    
    assert(!std::isnan(perturb.norm()));
    
    return perturb;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> BodyState::propagateAccelerometer(
        const BodyState &from_state, BodyState &to_state, const Quaternion &q_Bnext_Bcurrent) {
    double delta_t = from_state.timeTo(to_state);
    const Eigen::Vector3d global_gravity = from_state.calibration_->getGlobalGravity();
    
    Eigen::Matrix3d R_Bl_Bl1 = q_Bnext_Bcurrent.conjugate().toRotationMatrix();

    Eigen::Vector3d s_hat = delta_t/2.0 * (
            R_Bl_Bl1 * to_state.acceleration_estimate_ + from_state.acceleration_estimate_);
    Eigen::Vector3d y_hat = delta_t/2.0 * s_hat;
    
    Eigen::Matrix3d R_G_Bl = from_state.getOrientationInGlobalFrame().conjugate().toRotationMatrix();
    Eigen::Vector3d v_delta = R_G_Bl * s_hat + global_gravity*delta_t;

    Eigen::Vector3d p_delta = from_state.getVelocityInGlobalFrame()*delta_t + R_G_Bl*y_hat +
            0.5*global_gravity*(delta_t*delta_t);

    assert(!std::isnan(p_delta.norm()));
    assert(!std::isnan(v_delta.norm()));

    return std::make_pair(p_delta, v_delta);
}

Eigen::Matrix<double, 15, 15> BodyState::bodyStateTransitionMatrix(const Filter& filter, const BodyState& from_state,
        BodyState& to_state) {
    /// @todo Implement this
    return Eigen::Matrix<double, 15, 15>::Identity();
}

Eigen::Matrix<double, 15, 27> BodyState::imuCalibrationParamsTransitionMatrix(const Filter& filter,
        const BodyState& from_state, BodyState& to_state) {
    /// @todo Implement this
    return Eigen::Matrix<double, 15, 27>::Zero();
}

Eigen::Matrix<double, 15, 15> BodyState::propagationNoiseMatrix(const Filter& filter, const BodyState& from_state,
        BodyState& to_state, const Eigen::Matrix<double, 15, 15>& bodyStateTransitionMatrix) {
    /// @todo Implement this
    return Eigen::Matrix<double, 15, 15>::Zero();
}
