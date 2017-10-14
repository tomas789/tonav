//
// Created by Tomas Krejci on 5/12/16.
//

#include "filter.h"

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <functional>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <opencv2/core/core.hpp>

#include "body_state.h"
#include "chi_squared_ppm.h"
#include "calibration.h"
#include "camera_algorithms.h"
#include "filter_state.h"
#include "imu_item.h"
#include "imu_buffer.h"
#include "quaternion.h"
#include "quaternion_tools.h"
#include "stats.h"
#include "stats_timer.h"

namespace tonav {

Filter::Filter(
    std::shared_ptr<const Calibration> calibration,
    std::shared_ptr<const StateInitializer> state_initializer
)
    : calibration_(calibration), state_initializer_(state_initializer),
      feature_tracker_(calibration->getNumberOfFeaturesToExtract()), camera_algorithms_(this) {
}

void Filter::stepInertial(double time, const ImuItem &accel, const ImuItem &gyro) {
    if (!is_initialized_) {
        initialize(time, accel, gyro);
    }
    Eigen::Vector3d acceleration_estimate = computeAccelerationEstimate(accel.getVector());
    Eigen::Vector3d rotation_estimate = computeRotationEstimate(gyro.getVector(), acceleration_estimate);
    
    assert(state().body_state_.get());
    
    std::shared_ptr<BodyState> next_body_state = BodyState::propagate(
        *(state().body_state_.get()), time, rotation_estimate, acceleration_estimate);
    
    Eigen::Matrix<double, 56, 56> new_covar = BodyState::propagateCovariance(*this, *(state().body_state_.get()), *next_body_state, filter_covar_.block<56, 56>(0, 0));
    std::cout << "new_covar MIN: " << new_covar.minCoeff() << " | MAX: " << new_covar.maxCoeff() << std::endl;
    filter_covar_.block<56, 56>(0, 0) = new_covar;
    
    state().body_state_ = next_body_state;
}

void Filter::stepCamera(double time, cv::Mat &frame, const ImuBuffer::iterator &hint_gyro, const ImuBuffer::iterator &hint_accel) {
    if (!is_initialized_) {
        return;
    }
    augment(hint_gyro, hint_accel);
    CameraPose &last_camera_pose = state().poses().back();
    
    frame_rows_ = frame.rows;
    FeatureTracker::feature_track_list current_features_tracked;
    if (feature_tracker_.previous_frame_features_.keypoints().size() == 0) {
        assert(state().poses().size() == 1);
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
    
    FeatureTracker::feature_track_list features_to_rezidualize;
    for (std::size_t i = 0; i < features_tracked_.size(); ++i) {
        assert(features_tracked_[i]->posesTrackedCount() <= calibration_->getMaxCameraPoses() + 1 || features_tracked_[i]->wasUsedForResidualization());
        if (features_tracked_[i]->wasUsedForResidualization()) {
            continue;
        }
        if (features_tracked_[i]->isOutOfView()) {
            features_tracked_[i]->setWasUsedForResidualization();
            features_to_rezidualize.push_back(features_tracked_[i]);
        } else if (features_tracked_[i]->posesTrackedCount() == calibration_->getMaxCameraPoses() + 1) {
            features_tracked_[i]->setWasUsedForResidualization();
            features_tracked_[i]->revertLastPosition();
            last_camera_pose.decreaseActiveFeaturesCount(features_tracked_[i]->getFeatureId());
            features_to_rezidualize.push_back(features_tracked_[i]);
        }
    }
    
    // Perform MSCKF update step
    performUpdate(features_to_rezidualize, frame);

//    for (std::size_t i = 0; i < features_to_rezidualize.size(); ++i) {
//        features_to_rezidualize[i]->drawFeatureTrack(frame, cv::Scalar(0, 0, 255));
//    }
    
    pruneCameraPoses(features_to_rezidualize);
    features_tracked_ = current_features_tracked;
    
    std::cout << " 🚀 POSES IN STATE ";
    for (std::size_t i = 0; i < state().poses().size(); ++i) {
        std::cout << state().poses()[i].getCameraPoseId() << " ";
    }
    std::cout << std::endl;
    
    std::cout << *this << std::endl;
}

Eigen::Vector3d Filter::getCurrentPosition() {
    return state().getPositionInGlobalFrame();
}

Quaternion Filter::getCurrentAttitude() {
    return state().getOrientationInGlobalFrame();
}

Eigen::Vector3d Filter::getCurrentVelocity() {
    return state().getVelocityInGlobalFrame();
}

double Filter::getImageCaptureTime(double arrive_time) {
    return arrive_time + state().camera_delay_;
}

double Filter::getImageFirstLineCaptureTime(double arrive_time) {
    return arrive_time + state().camera_delay_ - std::abs(state().camera_readout_) / 2.0;
}

double Filter::getImageLastLineCaptureTime(double arrive_time) {
    return arrive_time + state().camera_delay_ + std::abs(state().camera_readout_) / 2.0;
}

bool Filter::isInitialized() const {
    return is_initialized_;
}

double Filter::time() const {
    if (!isInitialized()) {
        return NAN;
    }
    return state().time();
}

Quaternion Filter::getBodyToCameraRotation() const {
    return calibration_->getBodyToCameraRotation();
}

Eigen::Vector3d Filter::getPositionOfBodyInCameraFrame() const {
    return state().position_of_body_in_camera_;
}

void Filter::orientationCorrection(const Quaternion &orientation) {
    state().orientationCorrection(orientation);
}

void Filter::positionCorrection(const Eigen::Vector3d &position) {
    state().positionCorrection(position);
}

void Filter::velocityCorrection(const Eigen::Vector3d &velocity) {
    state().velocityCorrection(velocity);
}

const FilterState &Filter::state() const {
    return *filter_state_;
}

const Calibration &Filter::calibration() const {
    return *calibration_;
}

void Filter::setInitialBodyPositionInCameraFrame(const Eigen::Vector3d &position) {
    initial_body_position_in_camera_frame_ = position;
}

std::vector<Eigen::Vector3d> Filter::featurePointCloud() const {
    return feature_positions_;
}

const CameraAlgorithms &Filter::cameraAlgorithms() const {
    return camera_algorithms_;
}

FilterState &Filter::state() {
    return *filter_state_;
}

void Filter::initialize(double time, const ImuItem &accel, const ImuItem &gyro) {
    filter_state_ = std::make_shared<FilterState>(calibration_);
    
    state().bias_gyroscope_ = calibration_->getGyroscopeBias();
    state().bias_accelerometer_ = calibration_->getAccelerometerBias();
    state().gyroscope_shape_ = calibration_->getGyroscopeShapeMatrix();
    state().gyroscope_acceleration_sensitivity_ = calibration_->getGyroscopeAccelerationSensitivityMatrix();
    state().accelerometer_shape_ = calibration_->getAccelerometerShapeMatrix();
    state().position_of_body_in_camera_ = initial_body_position_in_camera_frame_;
    state().focal_point_ = calibration_->getCameraFocalPoint();
    state().optical_center_ = calibration_->getCameraOpticalCenter();
    state().radial_distortion_ = calibration_->getCameraRadialDistortionParams();
    state().tangential_distortion_ = calibration_->getCameraTangentialDistortionParams();
    state().camera_delay_ = calibration_->getCameraDelayTime();
    state().camera_readout_ = calibration_->getCameraReadoutTime();
    
    Eigen::Vector3d acceleration_estiamte = computeAccelerationEstimate(accel.getVector());
    Eigen::Vector3d rotation_estimate = computeRotationEstimate(gyro.getVector(), acceleration_estiamte);
    
    Quaternion attitude = state_initializer_->getOrientation();
    Eigen::Vector3d position = state_initializer_->getPosition();
    Eigen::Vector3d velocity = state_initializer_->getVelocity();
    
    state().body_state_ = std::make_shared<BodyState>(calibration_, time, rotation_estimate, acceleration_estiamte, attitude, position, velocity);
    
    Eigen::VectorXd covar_diag(56);
    covar_diag.segment<3>(0) = calibration_->getOrientationNoise();
    covar_diag.segment<3>(3) = calibration_->getPositionNoise();
    covar_diag.segment<3>(6) = calibration_->getVelocityNoise();
    covar_diag.segment<3>(9) = calibration_->getGyroscopeBiasNoise();
    covar_diag.segment<3>(12) = calibration_->getAccelerometerBiasNoise();
    covar_diag.segment<3>(15) = calibration_->getGyroscopeShapeMatrixNoise().block<1, 3>(0, 0).transpose();
    covar_diag.segment<3>(18) = calibration_->getGyroscopeShapeMatrixNoise().block<1, 3>(1, 0).transpose();
    covar_diag.segment<3>(21) = calibration_->getGyroscopeShapeMatrixNoise().block<1, 3>(2, 0).transpose();
    covar_diag.segment<3>(24) = calibration_->getGyroscopeAccelerationSensitivityMatrixNoise().block<1, 3>(0, 0).transpose();
    covar_diag.segment<3>(27) = calibration_->getGyroscopeAccelerationSensitivityMatrixNoise().block<1, 3>(1, 0).transpose();
    covar_diag.segment<3>(30) = calibration_->getGyroscopeAccelerationSensitivityMatrixNoise().block<1, 3>(2, 0).transpose();
    covar_diag.segment<3>(33) = calibration_->getAccelerometerShapeMatrixNoise().block<1, 3>(0, 0).transpose();
    covar_diag.segment<3>(36) = calibration_->getAccelerometerShapeMatrixNoise().block<1, 3>(1, 0).transpose();
    covar_diag.segment<3>(39) = calibration_->getAccelerometerShapeMatrixNoise().block<1, 3>(2, 0).transpose();
    covar_diag.segment<3>(42) = calibration_->getPositionOfBodyInCameraFrameNoise();
    covar_diag.segment<2>(45) = calibration_->getFocalPointNoise();
    covar_diag.segment<2>(47) = calibration_->getOpticalCenterNoise();
    covar_diag.segment<3>(49) = calibration_->getRadialDistortionNoise();
    covar_diag.segment<2>(52) = calibration_->getTangentialDistortionNoise();
    covar_diag(54) = calibration_->getCameraDelayTimeNoise();
    covar_diag(55) = calibration_->getCameraReadoutTimeNoise();
    
    filter_covar_.resize(56, 56);
    filter_covar_.setZero();
    filter_covar_ = covar_diag.asDiagonal();
    // filter_covar_ += 0.01*Eigen::MatrixXd::Identity(56, 56);
    
    Eigen::FullPivLU<Eigen::MatrixXd> full_piv_lu(filter_covar_);
    if (!full_piv_lu.isInvertible()) {
        std::cout << "Filter covariance matrix is not invertible." << std::endl;
        std::exit(1);
    } else {
        std::cout << "Filter covariance invertibility test passed." << std::endl;
    }
    
    is_initialized_ = true;
    
    assert(state().body_state_.get());
}

Eigen::Vector3d Filter::computeRotationEstimate(const Eigen::Vector3d &gyro, const Eigen::Vector3d &acceleration_estimate) const {
    Eigen::Matrix3d T_g_inv = state().gyroscope_shape_.inverse();
    Eigen::Matrix3d T_s = state().gyroscope_acceleration_sensitivity_;
    Eigen::Vector3d b_g = state().bias_gyroscope_;
    
    Eigen::Vector3d res = T_g_inv * (gyro - T_s * acceleration_estimate - b_g);
    assert(res.norm() < 1e100);
    
    return res;
}

Eigen::Vector3d Filter::computeAccelerationEstimate(const Eigen::Vector3d &accel) const {
    Eigen::Matrix3d T_a_inv = state().accelerometer_shape_.inverse();
    Eigen::Vector3d b_a = state().bias_accelerometer_;
    
    Eigen::Vector3d res = T_a_inv * (accel - b_a);
    assert(res.norm() < 1e100);
    
    return res;
}

void Filter::augment(const ImuBuffer::iterator &hint_gyro, const ImuBuffer::iterator &hint_accel) {
    CameraPose new_pose(*state().body_state_, hint_gyro, hint_accel);
    state().poses().pushBack(new_pose);
    std::cout << " 🚀 CREATE CAMERA POSE WITH ID " << new_pose.getCameraPoseId() << " | SIZE OF BUFFER " << state().poses().size() << std::endl;
    Eigen::MatrixXd new_covar_(filter_covar_.rows() + 9, filter_covar_.cols() + 9);
    Eigen::MatrixXd J_pi = Eigen::MatrixXd::Identity(9, filter_covar_.cols());
    new_covar_.block(0, 0, filter_covar_.rows(), filter_covar_.cols()) = filter_covar_;
    new_covar_.block(filter_covar_.rows(), 0, 9, filter_covar_.cols()) = J_pi * filter_covar_;
    new_covar_.block(0, filter_covar_.cols(), filter_covar_.rows(), 9) = new_covar_.block(filter_covar_.rows(), 0, 9, filter_covar_.cols()).transpose();
    new_covar_.block<9, 9>(filter_covar_.rows(), filter_covar_.cols()) = new_covar_.block(filter_covar_.rows(), 0, 9, filter_covar_.cols()) * J_pi.transpose();
    filter_covar_ = new_covar_;
}

void Filter::pruneCameraPoses(const FeatureTracker::feature_track_list &residualized_features) {
    std::size_t size_before = state().poses().size();
    for (std::size_t i = 0; i < residualized_features.size(); ++i) {
        CameraPoseBuffer::iterator it = std::end(state().poses());
        it = std::prev(it);
        
        for (std::size_t j = 0; j < residualized_features[i]->posesTrackedCount(); ++j) {
            it = std::prev(it);
            it->decreaseActiveFeaturesCount(residualized_features[i]->getFeatureId());
        }
    }
    
    std::size_t poses_deleted = 0;
    CameraPoseBuffer &poses = state().poses();
    while (!poses.empty() && poses.front().getActiveFeaturesCount() == 0) {
        poses.popFront();
        poses_deleted += 1;
    }
    
    Eigen::MatrixXd new_covar_(filter_covar_.rows() - 9 * poses_deleted, filter_covar_.cols() - 9 * poses_deleted);
    new_covar_.block<56, 56>(0, 0) = filter_covar_.block<56, 56>(0, 0);
    new_covar_.bottomRows(new_covar_.rows() - 56).leftCols<56>() = filter_covar_.bottomRows(new_covar_.rows() - 56).leftCols<56>();
    new_covar_.topRows<56>().rightCols(new_covar_.cols() - 56) = new_covar_.bottomRows(new_covar_.rows() - 56).leftCols<56>().transpose();
    new_covar_.bottomRows(new_covar_.rows() - 56).rightCols(new_covar_.cols() - 56) = filter_covar_.bottomRows(new_covar_.rows() - 56).rightCols(new_covar_.cols() - 56);
    filter_covar_ = new_covar_;
    
    std::cout << " 🚀 PRUNNING " << poses_deleted << " CAMERA POSES | SIZE BEFORE " << size_before << " | SIZE AFTER " << state().poses().size() << std::endl;
    
    assert(poses.size() <= calibration_->getMaxCameraPoses());
}

FeatureRezidualizationResult Filter::rezidualizeFeature(const FeatureTrack &feature_track, cv::Mat &frame) const {
    CameraPoseBuffer pose_buffer = state().poses();
    std::size_t track_length = feature_track.posesTrackedCount();
    std::size_t poses_in_state = pose_buffer.size();
    FeatureRezidualizationResult result(track_length, poses_in_state);
    
    std::pair<bool, Eigen::Vector3d> global_position_result = cameraAlgorithms().triangulateGlobalFeaturePosition(feature_track);
    bool global_position_success = global_position_result.first;
    Eigen::Vector3d global_position = global_position_result.second;
    result.setGlobalFeaturePosition(global_position);
    if (!global_position_success) {
        result.setIsInvalid();
//        std::cout << "Global position failed" << std::endl;
        return result;
    }
    
    bool vul = false;
    
    for (std::size_t j = 0; j < track_length; ++j) {
        Eigen::Vector2d z = feature_track[j];
        
        const CameraPose &pose = pose_buffer[poses_in_state - track_length + j - 1];
//        std::cout << "pose " << j << " has ID " << pose.getCameraPoseId() << std::endl;
        double pose_time = pose.time();
        double N = frame_rows_;
        double k = z(1) - N / 2;
        double feature_time = pose_time + state().camera_readout_ * k / N;
        
        ImuItem accel_item = ImuBuffer::interpolateAnyTime(pose_time, pose.accelHint());
        ImuItem gyro_item = ImuBuffer::interpolateAnyTime(pose_time, pose.gyroHint());
        
        Eigen::Vector3d acceleration_estimate = computeAccelerationEstimate(accel_item.getVector());
        Eigen::Vector3d rotation_estimate = computeRotationEstimate(gyro_item.getVector(), acceleration_estimate);
        std::shared_ptr<BodyState> body_state_for_feature = BodyState::propagate(pose.getBodyState(), feature_time, rotation_estimate, acceleration_estimate);
        //BodyState* body_state_for_feature = &pose.getBodyState()
        
        Eigen::Matrix3d R_C_B = getBodyToCameraRotation().toRotationMatrix();
        Eigen::Matrix3d R_Bj_G = body_state_for_feature->getOrientationInGlobalFrame().toRotationMatrix();
        Eigen::Vector3d p_Bj_G = body_state_for_feature->getPositionInGlobalFrame();
        Eigen::Vector3d v_Bj_G = body_state_for_feature->getVelocityInGlobalFrame();
        Eigen::Vector3d p_B_C = getPositionOfBodyInCameraFrame();

//        std::cout << "global position: " << global_position.transpose() << std::endl;
        
        Eigen::Vector3d p_f_C = R_C_B * R_Bj_G * (global_position - p_Bj_G) + p_B_C;
        
        if (p_f_C(2) < 0) {
            // Feature is behind camera
//            std::cout << "Feature is behind camera (p_f_C: [" << p_f_C.transpose() << "]^T" << std::endl;
            vul = true;
            //result.setIsInvalid();
            //return result;
        }
        Eigen::Vector2d z_hat = cameraAlgorithms().cameraProject(p_f_C);
        result.setPoseRezidual(j, z - z_hat);
        
        Eigen::Matrix<double, 2, 3> J_h = cameraAlgorithms().cameraProjectJacobian(p_f_C);
        Eigen::Matrix3d rotation_compound = R_C_B * R_Bj_G;
        Eigen::Matrix<double, 2, 3> M_i_j = J_h * rotation_compound;
        
        Eigen::Matrix<double, 3, 9> H_x_Bj_rhs_part;
        H_x_Bj_rhs_part.block<3, 3>(0, 0) = QuaternionTools::crossMatrix(global_position - p_Bj_G);
        H_x_Bj_rhs_part.block<3, 3>(0, 3) = -1 * Eigen::Matrix3d::Identity();
        H_x_Bj_rhs_part.block<3, 3>(0, 6) = (-1.0 * k * state().camera_readout_ / N) * Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 2, 9> H_x_Bj = M_i_j * H_x_Bj_rhs_part;
        result.setJacobianByCameraPose(j, H_x_Bj);
        //std::cout << "H_x_Bj(" << j << ") MIN: " << H_x_Bj.minCoeff() << " | MAX: " << H_x_Bj.maxCoeff() << std::endl;
        
        Eigen::Matrix<double, 2, 3> H_f_ij = M_i_j;
        result.setJacobianByFeaturePosition(j, H_f_ij);
        //std::cout << "H_f_ij(" << j << ") MIN: " << H_f_ij.minCoeff() << " | MAX: " << H_f_ij.maxCoeff() << std::endl;
        
        Eigen::Matrix<double, 2, 3> z_by_p_B_C = J_h;
        Eigen::Matrix3d cross_matrix_part;
        cross_matrix_part = QuaternionTools::crossMatrix(R_Bj_G * (global_position - p_Bj_G));
        Eigen::Vector2d z_by_td = J_h * R_C_B * (cross_matrix_part * rotation_estimate - R_Bj_G * v_Bj_G);
        Eigen::Vector2d z_by_tr = k / N * z_by_td;
        
        double u = p_f_C(0) / p_f_C(2);
        double v = p_f_C(1) / p_f_C(2);
        double r = u * u + v * v;
        double k1 = state().radial_distortion_(0);
        double k2 = state().radial_distortion_(1);
        double k3 = state().radial_distortion_(2);
        double t1 = state().tangential_distortion_(0);
        double t2 = state().tangential_distortion_(1);
        double fx = state().focal_point_(0);
        double fy = state().focal_point_(1);
        double dr = 1.0 + k1 * r + k2 * r * r + k3 * std::pow(r, 3.0);
        Eigen::Vector2d dt;
        dt(0) = 2.0 * u * v * t1 + (r + 2.0 * u * u) * t2;
        dt(1) = 2.0 * u * v * t2 + (r + 2.0 * v * v) * t1;
        
        Eigen::Vector2d h_by_fx;
        h_by_fx(0) = dr * u + dt(0);
        h_by_fx(1) = 0.0;
        Eigen::Vector2d h_by_fy;
        h_by_fy(0) = 0.0;
        h_by_fy(1) = dr * v + dt(1);
        Eigen::Vector2d h_by_ox = Eigen::Vector2d::UnitX();
        Eigen::Vector2d h_by_oy = Eigen::Vector2d::UnitY();
        Eigen::Vector2d h_by_k1;
        h_by_k1(0) = fx * u * r;
        h_by_k1(1) = fy * v * r;
        Eigen::Vector2d h_by_k2 = r * h_by_k1;
        Eigen::Vector2d h_by_k3 = r * h_by_k2;
        Eigen::Vector2d h_by_t1;
        h_by_t1(0) = fx * (2.0 * u * v);
        h_by_t1(1) = fy * (r + 2.0 * v * v);
        Eigen::Vector2d h_by_t2;
        h_by_t2(0) = fx * (r + 2.0 * u * u);
        h_by_t2(1) = fy * (2 * u * v);
        
        Eigen::Matrix<double, 2, 9> z_by_x_cam;
        z_by_x_cam.block<2, 1>(0, 0) = h_by_fx;
        z_by_x_cam.block<2, 1>(0, 1) = h_by_fy;
        z_by_x_cam.block<2, 1>(0, 2) = h_by_ox;
        z_by_x_cam.block<2, 1>(0, 3) = h_by_oy;
        z_by_x_cam.block<2, 1>(0, 4) = h_by_k1;
        z_by_x_cam.block<2, 1>(0, 5) = h_by_k2;
        z_by_x_cam.block<2, 1>(0, 6) = h_by_k3;
        z_by_x_cam.block<2, 1>(0, 7) = h_by_t1;
        z_by_x_cam.block<2, 1>(0, 8) = h_by_t2;
        
        Eigen::Matrix<double, 2, 14> H_c;
        H_c.block<2, 3>(0, 0) = z_by_p_B_C;
        H_c.block<2, 9>(0, 3) = z_by_x_cam;
        H_c.block<2, 1>(0, 12) = z_by_td;
        H_c.block<2, 1>(0, 13) = z_by_tr;
        
        // std::cout << "H_c(" << j << ") MIN: " << H_c.minCoeff() << " | MAX: " << H_c.maxCoeff() << std::endl;
        // std::cout << "z_by_p_B_C(" << j << ") MIN: " << z_by_p_B_C.minCoeff() << " | MAX: " << z_by_p_B_C.maxCoeff() << std::endl;
        //std::cout << "z_by_x_cam(" << j << ") MIN: " << z_by_x_cam.minCoeff() << " | MAX: " << z_by_x_cam.maxCoeff() << std::endl;
        // std::cout << "z_by_td(" << j << ") MIN: " << z_by_td.minCoeff() << " | MAX: " << z_by_td.maxCoeff() << std::endl;
        // std::cout << "z_by_tr(" << j << ") MIN: " << z_by_tr.minCoeff() << " | MAX: " << z_by_tr.maxCoeff() << std::endl;
        
        result.setJacobianByCameraParameters(j, H_c);
    }
    
    if (vul) {
        feature_track.drawFeatureTrack(frame, cv::Scalar(0, 0, 255), feature_track.posesTrackedCount());
//        std::cout << "Feature was at least once behind camera" << std::endl;
    } else {
        feature_track.drawFeatureTrack(frame, cv::Scalar(255, 255, 0), feature_track.posesTrackedCount());
//        std::cout << "Feature OK" << std::endl;
    }
    
    return result;
}

void Filter::performUpdate(const FeatureTracker::feature_track_list &features_to_rezidualize, cv::Mat &frame) {
    std::vector<Eigen::VectorXd> r_list;
    std::vector<Eigen::MatrixXd> H_list;
    std::size_t H_rows = 0;
    
    std::size_t num_poses = state().poses().size();
    feature_positions_.clear();
    
    int hist[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (const std::shared_ptr<FeatureTrack> &feature : features_to_rezidualize) {
        if (feature->posesTrackedCount() < 10) {
            hist[feature->posesTrackedCount()] += 1;
        }
    }
//    for (std::size_t i = 0; i < 10; ++i) {
//        std::cout << i << " " << hist[i] << " | ";
//    }
//    std::cout << std::endl;
    
    for (const std::shared_ptr<FeatureTrack> &feature : features_to_rezidualize) {
        if (feature->posesTrackedCount() <= 3) {
            continue;
        }
        
        FeatureRezidualizationResult rezidualization_result = rezidualizeFeature(*feature, frame);
        
        const Eigen::Matrix<double, Eigen::Dynamic, 3> &H_f = rezidualization_result.getJacobianByFeaturePosition();
        if (!rezidualization_result) {
            continue;
        } else {
//            std::cout << "Max residuum: " << rezidualization_result.getReziduals().maxCoeff() << " of feature with length " << feature->posesTrackedCount() << std::endl;
        }
        
        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(H_f.transpose());
        Eigen::MatrixXd A_t = lu_decomp.kernel().transpose();
        
        auto jac_by_state = rezidualization_result.getJacobianByState();
        
        // std::cout << "A_t MIN: " << A_t.minCoeff() << " | A_t MAX: " << A_t.maxCoeff() << std::endl;
        // std::cout << "jac_by_state MIN: " << jac_by_state.minCoeff() << " | jac_by_state MAX: " << jac_by_state.maxCoeff() << std::endl;
        
        // Eigen::MatrixXd A_t_H_f = A_t*H_f;
        // std::cout << "A_t_H_f MIN: " << A_t_H_f.minCoeff() << " | MAX: " << A_t_H_f.maxCoeff() << std::endl;
        
        Eigen::VectorXd r_0_i = A_t * rezidualization_result.getReziduals();
        Eigen::MatrixXd H_0_i = A_t * rezidualization_result.getJacobianByState();
        
        if (gatingTest(r_0_i, H_0_i)) {
            r_list.push_back(r_0_i);
            H_list.push_back(H_0_i);
            H_rows += H_0_i.rows();
            feature_positions_.push_back(rezidualization_result.getGlobalFeaturePosition());
        } else {
            // std::cout << "Feature is outlier." << std::endl;
        }
    }
    
    if (H_list.empty()) {
        std::cout << "No feature to update with. " << features_to_rezidualize.size() << std::endl;
        return;
    } else {
        std::cout << "Updating with " << H_list.size() << " out of " << features_to_rezidualize.size() << " features" << std::endl;
    }
    
    Eigen::VectorXd r_q;
    Eigen::MatrixXd T_H;
    Eigen::VectorXd r(H_rows);
    Eigen::MatrixXd H(H_rows, 56 + 9 * num_poses);
    std::size_t row_processed = 0;
    for (std::size_t i = 0; i < H_list.size(); ++i) {
        r.segment(row_processed, r_list[i].rows()) = r_list[i];
        H.block(row_processed, 0, H_list[i].rows(), H_list[i].cols()) = H_list[i];
        row_processed += r_list[i].rows();
    }
    
    std::cout << "r MIN: " << r.minCoeff() << " | MAX: " << r.maxCoeff() << std::endl;
    
    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> householder_qr = H.fullPivHouseholderQr();
    Eigen::MatrixXd R = householder_qr.matrixQR().triangularView<Eigen::Upper>();
    Eigen::MatrixXd Q = householder_qr.matrixQ();
    
    std::size_t nonZeros = 0;
    for (; nonZeros < R.rows(); ++nonZeros) {
        if (R.block(nonZeros, 0, 1, R.cols()).isZero(1e-6)) {
            break;
        }
    }
    
    T_H = R.topRows(nonZeros);
    auto Q_1 = Q.leftCols(nonZeros);
    r_q = Q_1.transpose() * r;
    
    updateState(T_H, r_q, H, r);
}

bool Filter::gatingTest(const Eigen::VectorXd &r_0_i, const Eigen::MatrixXd H_0_i) {
    double sigma = 1.0;
    // std::cout << "H_0_i MIN: " << H_0_i.minCoeff() << " | H_0_i MAX: " << H_0_i.maxCoeff() << std::endl;
    auto inner = H_0_i * filter_covar_ * H_0_i.transpose() + sigma * Eigen::MatrixXd::Identity(H_0_i.rows(), H_0_i.rows());
    assert(inner.rows() == inner.cols());
    // std::cout << "INNER MIN: " << inner.minCoeff() << " | INNER MAX: " << inner.maxCoeff() << std::endl;
    Eigen::FullPivLU<Eigen::MatrixXd> full_piv_lu(inner);
    if (!full_piv_lu.isInvertible()) {
        std::cout << "Inner ix " << inner.cols() << "x" << inner.rows() << " with rank " << full_piv_lu.rank() << std::endl;
        // std::cout << "Not invertible" << std::endl;
        return false;
    }
    auto inner_inverted = inner.inverse();
    Eigen::MatrixXd gamma_i = r_0_i.transpose() * inner_inverted * r_0_i;
    bool test_passed = gamma_i(0, 0) < CHI_SQUARED_PPM_VALUES[r_0_i.rows()];
    if (!test_passed) {
        // std::cout << "Test failed with gamma_i: " << gamma_i(0, 0) << " threshold: " << CHI_SQUARED_PPM_VALUES[r_0_i.cols()] << std::endl;
    }
    return test_passed;
}

void Filter::updateState(const Eigen::MatrixXd &T_H, const Eigen::VectorXd &r_q, const Eigen::MatrixXd &H, const Eigen::VectorXd &r) {
    double sigma = calibration_->getImageNoiseVariance();
    double sigma_squared = sigma * sigma;
    
    /*
    Eigen::MatrixXd K_big = filter_covar_*H.transpose()*(H*filter_covar_*H.transpose() + sigma_squared*Eigen::MatrixXd::Identity(H.rows(), H.rows())).inverse();
    Eigen::VectorXd delta_x_big = K_big * r;
    std::cout << "r: [" << r.transpose() << "]^T" << std::endl;
    std::cout << "K_big (velocity radky): \n" << K_big.block(6, 0, 3, K_big.cols()) << std::endl;
    std::cout << "delta_x_big: [" << delta_x_big.transpose() << "]^T" << std::endl;
    filter_state_->updateWithStateDelta(delta_x_big);
    return;
     */
    
    Eigen::MatrixXd R_q = sigma_squared * Eigen::MatrixXd::Identity(T_H.rows(), T_H.rows());
    // Kalman gain
    Eigen::MatrixXd K = filter_covar_ * T_H.transpose() * (T_H * filter_covar_ * T_H.transpose() + R_q).inverse();
    Eigen::MatrixXd I_beta = Eigen::MatrixXd::Identity(T_H.cols(), T_H.cols());
    Eigen::MatrixXd covar_prop_part = I_beta - K * T_H;
    //filter_covar_ = covar_prop_part*filter_covar_*covar_prop_part.transpose() + K*R_q*K.transpose();
    Eigen::VectorXd delta_x = K * r_q;
    
    filter_state_->updateWithStateDelta(delta_x);
}

std::ostream &operator<<(std::ostream &out, tonav::Filter &filter) {
    FilterState &state = filter.state();
    Eigen::IOFormat formatter(4, 0, ", ", "\n", "[", "]");
    out << std::setprecision(4);
    Eigen::Vector3d euler = state.getOrientationInGlobalFrame().toRotationMatrix().eulerAngles(0, 1, 2);
    out << "Euler angles:    " << euler.transpose().format(formatter) << " ± (" << filter.filter_covar_(0, 0) << ", " << filter.filter_covar_(1, 1) << ", " << filter.filter_covar_(2, 2) << ")" << std::endl;
    out << "Rotation:        " << state.getOrientationInGlobalFrame().coeffs().transpose().format(formatter)
        << std::endl;
    out << "Position:        " << state.getPositionInGlobalFrame().transpose().format(formatter) << " ± (" << filter.filter_covar_(3, 3) << ", " << filter.filter_covar_(4, 4) << ", " << filter.filter_covar_(5, 5) << ")" << std::endl;
    out << "Velocity:        " << state.getVelocityInGlobalFrame().transpose().format(formatter) << " ± (" << filter.filter_covar_(6, 6) << ", " << filter.filter_covar_(7, 7) << ", " << filter.filter_covar_(8, 8) << ")" << std::endl;
    out << "Gyro bias:       " << state.bias_gyroscope_.transpose().format(formatter) << std::endl;
    out << "Accel bias:      " << state.bias_accelerometer_.transpose().format(formatter) << std::endl;
    out << "Gyro shape:      " << std::endl << state.gyroscope_shape_.format(formatter) << std::endl;
    out << "G-Sensitivity:   " << std::endl << state.gyroscope_acceleration_sensitivity_.format(formatter) << std::endl;
    out << "Accel shape:     " << std::endl << state.accelerometer_shape_.format(formatter)
        << std::endl;
    out << "p_B_C:           " << state.position_of_body_in_camera_.transpose().format(formatter) << " ± (" << filter.filter_covar_(43, 43) << ", " << filter.filter_covar_(44, 44) << ", " << filter.filter_covar_(45, 45) << ")" << std::endl;
    out << "Focal length:    " << state.focal_point_.transpose().format(formatter) << " ± (" << filter.filter_covar_(46, 46) << ", " << filter.filter_covar_(47, 47) << ")" << std::endl;
    out << "Optical center:  " << state.optical_center_.transpose().format(formatter) << " ± (" << filter.filter_covar_(48, 48) << ", " << filter.filter_covar_(49, 49) << ")" << std::endl;
    out << "Radial dist:     " << state.radial_distortion_.transpose().format(formatter) << " ± (" << filter.filter_covar_(50, 50) << ", " << filter.filter_covar_(51, 51) << ")" << std::endl;
    out << "Tangential dist: " << state.tangential_distortion_.transpose().format(formatter) << " ± (" << filter.filter_covar_(52, 52) << ", " << filter.filter_covar_(53, 53) << ")" << std::endl;
    out << "Cam delay:       " << state.camera_delay_ << " ± (" << filter.filter_covar_(54, 54) << ")" << std::endl;
    out << "Cam readout:     " << state.camera_readout_ << " ± (" << filter.filter_covar_(55, 55) << ")" << std::endl;
    
    return out;
}

}
