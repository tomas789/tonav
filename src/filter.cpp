//
// Created by Tomas Krejci on 5/12/16.
//

#include "filter.h"

#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>

#include "calibration.h"
#include "filter_state.h"
#include "imu_item.h"

Filter::Filter(const Calibration &calibration)
: calibration_(calibration), filter_state_(calibration_.getMaxCameraPoses()) {
}

void Filter::initialize() {
    initializeBodyFrame();
    initializeImuCalibration();
    initializeCameraCalibration();
    initializeBodyPoses();

    filter_state_.getRotationEstimateBlock().setZero();
    filter_state_.getAccelerationEstimateBlock().setZero();

    // std::cout << filter_state_ << std::endl;

    filter_covar_.setIdentity();
}

void Filter::stepInertial(double timedelta, const ImuItem &accel, const ImuItem &gyro) {
    FilterState new_state = filter_state_.deriveNewStateForImuPropagation();
    FilterState& old_state = filter_state_;

    BodyState body_state_new = propagateBodyState(old_state.getBodyStateRef(), timedelta, accel, gyro);
    new_state.getBodyStateRef() = body_state_new;

    old_state = new_state;
    // std::cout << filter_state_ << std::endl;
}

void Filter::stepCamera(double timedelta, cv::Mat &frame) {
    augment();
    CameraPose& last_camera_pose = filter_state_.poses().back();
    
    FeatureTracker::feature_track_list current_features_tracked;
    if (feature_tracker_.previous_frame_features_.keypoints().size() == 0) {
        assert(filter_state_.poses().size() == 1);
    }
    current_features_tracked = feature_tracker_.processImage(features_tracked_, frame);
    last_camera_pose.setActiveFeaturesCount(current_features_tracked.size());
    for (std::size_t i = 0; i < current_features_tracked.size(); ++i) {
        if (current_features_tracked[i]->wasUsedForResidualization()) {
            last_camera_pose.rememberFeatureId(current_features_tracked[i]->getFeatureId());
            last_camera_pose.decreaseActiveFeaturesCount(current_features_tracked[i]->getFeatureId());
        } else {
            last_camera_pose.rememberFeatureId(current_features_tracked[i]->getFeatureId());
        }
    }

    FeatureTracker::feature_track_list features_to_residualize;
    for (std::size_t i = 0; i < features_tracked_.size(); ++i) {
        assert(features_tracked_[i]->posesTrackedCount() <= calibration_.getMaxCameraPoses() + 1 || features_tracked_[i]->wasUsedForResidualization());
        if (features_tracked_[i]->wasUsedForResidualization()) {
            continue;
        }
        if (features_tracked_[i]->isOutOfView()) {
            features_tracked_[i]->setWasUsedForResidualization();
            features_to_residualize.push_back(features_tracked_[i]);
        } else if (features_tracked_[i]->posesTrackedCount() == calibration_.getMaxCameraPoses() + 1) {
            features_tracked_[i]->setWasUsedForResidualization();
            features_tracked_[i]->revertLastPosition();
            last_camera_pose.decreaseActiveFeaturesCount(features_tracked_[i]->getFeatureId());
            features_to_residualize.push_back(features_tracked_[i]);
        }
    }
    
    
    
    pruneCameraPoses(features_to_residualize);
    features_tracked_ = current_features_tracked;
    
    std::cout << "Cam poses: " << filter_state_.poses().size() << std::endl;
}

BodyState Filter::propagateBodyState(const BodyState& body_state_old, double timedelta,
        const ImuItem& accel, const ImuItem& gyro) {
    BodyState body_state_new;
    
    body_state_new.getGyroscopeBiasBlock() = body_state_old.getGyroscopeBiasBlock();
    body_state_new.getAccelerometerBiasBlock() = body_state_new.getAccelerometerBiasBlock();
    
    propagateRotation(body_state_old, body_state_new, timedelta, accel, gyro);
    propagateVelocityAndPosition(body_state_old, body_state_new, timedelta, accel, gyro);
    
    return body_state_new;
}

void Filter::propagateRotation(const BodyState& old_state, BodyState& new_state, double timedelta,
        const ImuItem& accel, const ImuItem& gyro) {
    Eigen::Vector3d rotation_measured;
    rotation_measured << gyro.getX(), gyro.getY(), gyro.getZ();
    Eigen::Vector3d accel_measured;
    accel_measured << accel.getX(), accel.getY(), accel.getZ();

    Eigen::Matrix3d gyro_shape = unvectorizeMatrix(filter_state_.getGyroscopeShapeVectorizedBlock());
    Eigen::Matrix3d accel_shape = unvectorizeMatrix(filter_state_.getAccelerometerShapeVectorizedBlock());
    Eigen::Matrix3d g_sensitivity = unvectorizeMatrix(filter_state_.getGSensitivityVectorizedBlock());

    Eigen::Vector3d accel_estimate = accel_shape.inverse() * (accel_measured - new_state.getAccelerometerBiasBlock());
    Eigen::Vector3d rotation_estimate_new;
    rotation_estimate_new = gyro_shape.inverse() * (
            rotation_measured - g_sensitivity*accel_estimate - new_state.getGyroscopeBiasBlock()
        );
    new_state.getRotationEstimateRef() = rotation_estimate_new;

    Eigen::Vector4d q0;
    q0 << 0, 0, 0, 1;

    Eigen::Vector4d k1 = 0.5 * Filter::omegaMatrix(old_state.getRotationEstimateRef()) * q0;
    Eigen::Matrix4d omega_mat_k2_k3 = Filter::omegaMatrix(
            (old_state.getRotationEstimateRef() + new_state.getRotationEstimateRef())/ 2
        );
    Eigen::Vector4d k2 = 0.5 * omega_mat_k2_k3 * (q0 + (timedelta/2.0)*k1);
    Eigen::Vector4d k3 = 0.5 * omega_mat_k2_k3 * (q0 + (timedelta/2.0)*k2);
    Eigen::Vector4d k4 = 0.5 * Filter::omegaMatrix(new_state.getRotationEstimateRef()) * (q0 + timedelta*k3);

    Eigen::Vector4d trans_to_new_frame_vec = q0 + timedelta/6.0 * (k1 + 2*k2 + 2*k3 + k4);

    Eigen::Quaterniond to_old_frame = old_state.getRotationQuaternion();
    Eigen::Quaterniond trans_to_new_frame(
            trans_to_new_frame_vec(3, 0), trans_to_new_frame_vec(0, 0), trans_to_new_frame_vec(1, 0),
            trans_to_new_frame_vec(2, 0)
        );
    trans_to_new_frame.normalize();
    new_state.getRotationToThisFrameRef() = trans_to_new_frame;
    Eigen::Quaterniond to_new_frame = trans_to_new_frame * to_old_frame;
    new_state.setRotationQuaternion(to_new_frame);
}

void Filter::propagateVelocityAndPosition(
        const BodyState& old_state, BodyState& new_state, double timedelta, const ImuItem &accel, const ImuItem &gyro) {
    Eigen::Matrix3d accelerometer_shape = Filter::unvectorizeMatrix(filter_state_.getAccelerometerShapeVectorizedBlock());
    Eigen::Vector3d accelerometer_measurement;
    accelerometer_measurement << accel.getX(), accel.getY(), accel.getZ();
    new_state.getAccelerationEstimateRef() = accelerometer_shape.inverse() * (
            accelerometer_measurement - new_state.getAccelerometerBiasBlock()
        );

    Eigen::Matrix3d rotation_backwards = new_state.getRotationToThisFrameRef().toRotationMatrix().transpose();

    Eigen::Vector3d s_estimate = timedelta/2.0 * (
            rotation_backwards*new_state.getAccelerationEstimateRef() + old_state.getAccelerationEstimateRef()
        );
    Eigen::Vector3d y_estimate = timedelta/2.0 * s_estimate;

    Eigen::Matrix3d rotation_from_body_to_global = old_state.getRotationQuaternion().toRotationMatrix().transpose();
    new_state.getVelocityBlock() =
            old_state.getVelocityBlock() + rotation_from_body_to_global * s_estimate + getGlobalGravity()*timedelta;

    new_state.getPositionBlock() =
            old_state.getPositionBlock() + old_state.getVelocityBlock()*timedelta +
            rotation_from_body_to_global*y_estimate + 0.5 * getGlobalGravity()*timedelta*timedelta;
}

void Filter::setGlobalGravity(Eigen::Vector3d gravity) {
    global_gravity_ = gravity;
}

Eigen::Vector3d Filter::getGlobalGravity() const {
    return global_gravity_;
}

Eigen::Vector3d Filter::getCurrentPosition() {
    return filter_state_.getPositionBlock();
}

Eigen::Quaterniond Filter::getCurrentAttitude() {
    return filter_state_.getRotationQuaternion();
}

double Filter::getImageCaptureTime(double arrive_time) {
    return arrive_time + filter_state_.getCameraDelayTimeRef();
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

void Filter::augment() {
    CameraPose pose;
    BodyState& body_state = filter_state_.getBodyStateRef();
    pose.getRotationForBodyPoseBlock() = body_state.getRotationBlock();
    pose.getPositionForBodyPoseBlock() = body_state.getPositionBlock();
    pose.getVelocityForBodyPoseBlock() = body_state.getVelocityBlock();
    filter_state_.poses().addNewCameraPose(pose);
}

void Filter::pruneCameraPoses(const FeatureTracker::feature_track_list& residualized_features) {
    for (std::size_t i = 0; i < residualized_features.size(); ++i) {
        CameraPoseBuffer::iterator it = std::end(filter_state_.poses());
        it = std::prev(it);
        
        for (std::size_t j = 0; j < residualized_features[i]->posesTrackedCount(); ++j) {
            it = std::prev(it);
            it->decreaseActiveFeaturesCount(residualized_features[i]->getFeatureId());
        }
    }
    
    CameraPoseBuffer& poses = filter_state_.poses();
    while (!poses.empty() && poses.front().getActiveFeaturesCount() == 0) {
        poses.deleteOldestCameraPose();
    }
    
    assert(poses.size() <= calibration_.getMaxCameraPoses());
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

Eigen::Vector3d Filter::triangulateGlobalFeaturePosition(const FeatureTrack &feature_track) {
    std::size_t n = feature_track.posesTrackedCount();
    std::size_t max_iter = static_cast<std::size_t>(calibration_.getMaxTriangulationIterations());
    CameraPoseBuffer& poses = filter_state_.poses();
    
    /** \f$ f \f$ */
    Eigen::Matrix<double, Eigen::Dynamic, 1> measurement_errors;
    measurement_errors.resize(2*n, Eigen::NoChange);
    /** \f$ J_{f_i} \f$ */
    Eigen::Matrix<double, Eigen::Dynamic, 3> measurement_error_jacobian;
    measurement_error_jacobian.resize(2*n, Eigen::NoChange);
    
    /** @todo Make better initial guess */
    Eigen::Vector3d parameters_est;
    parameters_est << 1, 1, 1;
    
    for (std::size_t i = 0; i < max_iter; ++i) {
        CameraPoseBuffer::iterator it = std::end(poses);
        it = std::prev(it, 2);
        
        for (std::size_t pose_counter = 0; pose_counter < n; ++pose_counter) {
            /** \f$ z_i \f$ */
            const Eigen::Vector2d& position = feature_track[n - pose_counter - 1];
            CameraPose& pose = *it;
            
            
            
            it = std::prev(it);
        }
    }
    
    
}



