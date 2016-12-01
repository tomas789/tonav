#include "body_state.h"

#include <Eigen/Core>
#include <memory>
#include <iostream>

#include "calibration.h"
#include "quaternion_tools.h"

BodyState::BodyState(std::shared_ptr<const Calibration> calibration, double time, Eigen::Vector3d rotation_estimate,
        Eigen::Vector3d acceleration_estimate, Eigen::Quaterniond q_B_G, Eigen::Vector3d p_B_G, Eigen::Vector3d v_B_G) {
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
    rotation_to_this_frame_ = Eigen::Quaterniond::Identity();
}

std::shared_ptr<BodyState> BodyState::propagate(const BodyState& from_state, double time, Eigen::Vector3d rotation_estimate,
        Eigen::Vector3d acceleration_estimate) {
    std::shared_ptr<BodyState> to_state = std::make_shared<BodyState>(from_state);
    to_state->time_ = time;
    to_state->rotation_estimate_ = rotation_estimate;
    to_state->acceleration_estimate_ = acceleration_estimate;
    Eigen::Quaterniond q_Bcurrent_Bnext = BodyState::propagateGyroscope(from_state, *to_state);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> p_and_v_delta = BodyState::propagateAccelerometer(
            from_state, *to_state, q_Bcurrent_Bnext.conjugate());
    Eigen::Vector3d p_delta = p_and_v_delta.first;
    Eigen::Vector3d v_delta = p_and_v_delta.second;

    // Use * instead of *= because quaternion product is not commutative.
    to_state->q_B_G_ = q_Bcurrent_Bnext.conjugate() * from_state.q_B_G_;
    to_state->p_B_G_ = from_state.p_B_G_ + p_delta;
    to_state->v_B_G_ = from_state.v_B_G_ + v_delta;
        
    return to_state;
}

double BodyState::time() const {
    return time_;
}

double BodyState::timeTo(const BodyState &to_state) const {
    return to_state.time_ - time_;
}

const Eigen::Quaterniond& BodyState::getOrientationInGlobalFrame() const {
    return q_B_G_;
}

const Eigen::Vector3d& BodyState::getPositionInGlobalFrame() const {
    return p_B_G_;
}

const Eigen::Vector3d& BodyState::getVelocityInGlobalFrame() const {
    return v_B_G_;
}

Eigen::Quaterniond BodyState::propagateGyroscope(const BodyState &from_state, BodyState &to_state) {
    Eigen::Vector4d q0;
    q0 << 1.0, 0.0, 0.0, 0.0;
    double delta_t = from_state.timeTo(to_state);
    const Eigen::Vector3d& rotation_est_prev = from_state.rotation_estimate_;
    const Eigen::Vector3d& rotation_est = to_state.rotation_estimate_;
    Eigen::Vector3d rotation_est_mid = (rotation_est_prev + rotation_est) / 2.0;

    Eigen::Vector4d k1 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est_prev) * q0;
    Eigen::Vector4d k2 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est_mid) * (q0 + delta_t/2.0 * k1);
    Eigen::Vector4d k3 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est_mid) * (q0 + delta_t/2.0 * k2);
    Eigen::Vector4d k4 = 0.5 * QuaternionTools::bigOmegaMatrix(rotation_est) * (q0 + delta_t * k3);

    Eigen::Vector4d perturb_vec = q0 + delta_t/6.0 * (k1 + 2.0*k2 + 2.0*k3 + k4);
    Eigen::Quaterniond perturb(perturb_vec(0), perturb_vec(1), perturb_vec(2), perturb_vec(3));
    perturb.normalize();
    
    assert(!std::isnan(perturb.norm()));
    
    return perturb;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> BodyState::propagateAccelerometer(
        const BodyState &from_state, BodyState &to_state, const Eigen::Quaterniond &q_Bnext_Bcurrent) {
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
