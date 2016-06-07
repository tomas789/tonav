//
// Created by Tomas Krejci on 5/12/16.
//

#include "filter.h"

#include <iostream>
#include <opencv2/core/core.hpp>

#include "calibration.h"
#include "filter_state.h"
#include "imu_item.h"

Filter::Filter(const Calibration &calibration) : calibration_(calibration), filter_state_(calibration.getMaxCameraPoses()) {
}

void Filter::initialize() {
    initializeBodyFrame();
    initializeImuCalibration();
    initializeCameraCalibration();
    initializeBodyPoses();

    filter_state_.getRotationEstimateBlock().setZero();
    filter_state_.getAccelerationEstimateBlock().setZero();

    std::cout << filter_state_ << std::endl;

    filter_covar_.setIdentity();
}

void Filter::stepInertial(double timedelta, const ImuItem &accel, const ImuItem &gyro) {
    FilterState new_state = filter_state_.deriveNewStateForImuPropagation();
    FilterState& old_state = filter_state_;

    propagateRotation(old_state, new_state, timedelta, accel, gyro);
    propagateVelocityAndPosition(old_state, new_state, timedelta, accel, gyro);

    old_state = new_state;
    std::cout << filter_state_ << std::endl;
}

void Filter::stepCamera(double timedelta, cv::Mat &frame) {
    FeatureTracker::feature_track_list current_features_tracked;
    current_features_tracked = feature_tracker_.processImage(features_tracked_, frame);

    FeatureTracker::feature_track_list out_of_view_features;
    for (std::size_t i = 0; i < features_tracked_.size(); ++i) {
        if (features_tracked_[i]->isOutOfView()) {
            out_of_view_features.push_back(features_tracked_[i]);
        }
    }

    features_tracked_ = current_features_tracked;
}

void Filter::propagateRotation(FilterState &old_state, FilterState &new_state, double timedelta, const ImuItem &accel,
        const ImuItem &gyro) {
    Eigen::Vector3d rotation_measured;
    rotation_measured << gyro.getX(), gyro.getY(), gyro.getZ();
    Eigen::Vector3d accel_measured;
    accel_measured << accel.getX(), accel.getY(), accel.getZ();

    Eigen::Matrix3d gyro_shape = unvectorizeMatrix(new_state.getGyroscopeShapeVectorizedBlock());
    Eigen::Matrix3d accel_shape = unvectorizeMatrix(new_state.getAccelerometerShapeVectorizedBlock());
    Eigen::Matrix3d g_sensitivity = unvectorizeMatrix(new_state.getGSensitivityVectorizedBlock());

    Eigen::Vector3d accel_estimate = accel_shape.inverse() * (accel_measured - new_state.getAccelerometerBiasBlock());
    Eigen::Vector3d rotation_estimate_new;
    rotation_estimate_new = gyro_shape.inverse() * (
            rotation_measured - g_sensitivity*accel_estimate - new_state.getGyroscopeBiasBlock()
        );
    new_state.getRotationEstimateBlock() = rotation_estimate_new;

    Eigen::Vector4d q0;
    q0 << 0, 0, 0, 1;

    Eigen::Vector4d k1 = 0.5 * Filter::omegaMatrix(old_state.getRotationEstimateBlock()) * q0;
    Eigen::Matrix4d omega_mat_k2_k3 = Filter::omegaMatrix(
            (old_state.getRotationEstimateBlock() + new_state.getRotationEstimateBlock())/ 2
        );
    Eigen::Vector4d k2 = 0.5 * omega_mat_k2_k3 * (q0 + (timedelta/2.0)*k1);
    Eigen::Vector4d k3 = 0.5 * omega_mat_k2_k3 * (q0 + (timedelta/2.0)*k2);
    Eigen::Vector4d k4 = 0.5 * Filter::omegaMatrix(new_state.getRotationEstimateBlock()) * (q0 + timedelta*k3);

    Eigen::Vector4d trans_to_new_frame_vec = q0 + timedelta/6.0 * (k1 + 2*k2 + 2*k3 + k4);

    Eigen::Quaterniond to_old_frame = old_state.getRotationQuaternion();
    Eigen::Quaterniond trans_to_new_frame(
            trans_to_new_frame_vec(3, 0), trans_to_new_frame_vec(0, 0), trans_to_new_frame_vec(1, 0),
            trans_to_new_frame_vec(2, 0)
        );
    trans_to_new_frame.normalize();
    new_state.setRotationToThisFrame(trans_to_new_frame);
    Eigen::Quaterniond to_new_frame = trans_to_new_frame * to_old_frame;
    new_state.setRotationQuaternion(to_new_frame);
}

void Filter::propagateVelocityAndPosition(
        FilterState &old_state, FilterState &new_state, double timedelta, const ImuItem &accel, const ImuItem &gyro) {
    Eigen::Matrix3d accelerometer_shape = Filter::unvectorizeMatrix(new_state.getAccelerometerShapeVectorizedBlock());
    Eigen::Vector3d accelerometer_measurement;
    accelerometer_measurement << accel.getX(), accel.getY(), accel.getZ();
    new_state.getAccelerationEstimateBlock() = accelerometer_shape.inverse() * (
            accelerometer_measurement - new_state.getAccelerometerBiasBlock()
        );

    Eigen::Matrix3d rotation_backwards = new_state.getRotationToThisFrame().toRotationMatrix().transpose();

    Eigen::Vector3d s_estimate = timedelta/2.0 * (
            rotation_backwards*new_state.getAccelerationEstimateBlock() + old_state.getAccelerationEstimateBlock()
        );
    Eigen::Vector3d y_estimate = timedelta/2.0 * s_estimate;

    Eigen::Matrix3d rotation_from_body_to_global = old_state.getRotationQuaternion().toRotationMatrix().transpose();
    new_state.getVelocityBlock() =
            old_state.getVelocityBlock() + rotation_from_body_to_global * s_estimate + getGlobalGravity()*timedelta;

    new_state.getPositionBlock() =
            old_state.getPositionBlock() + old_state.getVelocityBlock()*timedelta +
            rotation_from_body_to_global*y_estimate + 0.5 * getGlobalGravity()*timedelta*timedelta;
}

Eigen::Vector3d Filter::getGlobalGravity() const {
    Eigen::Vector3d gravity;
    gravity << 0, 0, -1;
    return gravity;
}

Eigen::Vector3d Filter::getCurrentPosition() {
    return filter_state_.getPositionBlock();
}

Eigen::Quaterniond Filter::getCurrentAttitude() {
    return filter_state_.getRotationQuaternion();
}

void Filter::initializeBodyFrame() {
    Eigen::Quaterniond initial_rotation = Eigen::Quaterniond::Identity();
    filter_state_.getRotationBlock() << initial_rotation.vec(), initial_rotation.w();

    filter_state_.getPositionBlock().setZero();
    filter_state_.getVelocityBlock().setZero();
    filter_state_.getGyroscopeBiasBlock().setZero();
    filter_state_.getAccelerometerBiasBlock().setZero();
}

void Filter::initializeImuCalibration() {
    auto gyroscope_shape_vectorized = Filter::vectorizeMatrix(calibration_.getGyroscopeShapeMatrix());
    auto g_senstivity_vectorized = Filter::vectorizeMatrix(calibration_.getGSensitivityMatrix());
    auto accelerometer_shape_vectorized = Filter::vectorizeMatrix(calibration_.getAccelerometerShapeMatrix());

    filter_state_.getGyroscopeShapeVectorizedBlock() = gyroscope_shape_vectorized;
    filter_state_.getGSensitivityVectorizedBlock() = g_senstivity_vectorized;
    filter_state_.getAccelerometerShapeVectorizedBlock() = accelerometer_shape_vectorized;
}

void Filter::initializeCameraCalibration() {
    filter_state_.getCameraToBodyOffsetBlock() = calibration_.getCameraToBodyOffset();
    filter_state_.getFocalLengthXRef() = calibration_.getFocalLengthX();
    filter_state_.getFocalLengthYRef() = calibration_.getFocalLengthY();
    filter_state_.getOpticalCenterXRef() = calibration_.getOpticalCenterX();
    filter_state_.getOpticalCenterYRef() = calibration_.getOpticalCenterY();
    filter_state_.getRadialDistortionParametersBlock() = calibration_.getRadialDistortionParameters();
    filter_state_.getTangentialDistortionParametersBlock() = calibration_.getTangentialDistortionParameters();
    filter_state_.getCameraDelayTimeRef() = calibration_.getCameraDelayTime();
    filter_state_.getCameraReadoutTimeRef() = calibration_.getCameraReadoutTime();
}

void Filter::initializeBodyPoses() {
}

Eigen::Matrix<double, 9, 1> Filter::vectorizeMatrix(const Eigen::Matrix<double, 3, 3> &mat) {
    Eigen::Matrix<double, 9, 1> vec;
    vec.block<3, 1>(0, 0) = mat.row(0);
    vec.block<3, 1>(3, 0) = mat.row(1);
    vec.block<3, 1>(6, 0) = mat.row(2);
    return vec;
}

Eigen::Matrix<double, 3, 3> Filter::unvectorizeMatrix(Eigen::Block<FilterState::StateType, 9, 1> vec) {
    Eigen::Matrix<double, 3, 3> mat;
    mat.row(0) = vec.block<3, 1>(0, 0);
    mat.row(1) = vec.block<3, 1>(3, 0);
    mat.row(2) = vec.block<3, 1>(6, 0);
    return mat;
}

Eigen::Matrix4d Filter::omegaMatrix(const Eigen::Vector3d vec) {
    Eigen::Matrix4d mat;
    mat.block<3, 3>(0, 0) = Filter::crossMatrix(-1*vec);
    mat.block<1, 3>(3, 0) = -1*(vec.transpose());
    mat.block<3, 1>(0, 3) = vec;
    mat(3, 3) = 0;
    return mat;
}

Eigen::Matrix3d Filter::crossMatrix(const Eigen::Vector3d vec) {
    Eigen::Matrix3d mat;
    mat <<         0, -1*vec(2),    vec(1),
              vec(2),         0, -1*vec(0),
           -1*vec(1),    vec(0),         0;
    return mat;
}








