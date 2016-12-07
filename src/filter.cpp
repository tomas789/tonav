//
// Created by Tomas Krejci on 5/12/16.
//

#include "filter.h"

#include <cmath>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <functional>
#include <unsupported/Eigen/LevenbergMarquardt>
#include <Eigen/QR>

#include "chi_squared_ppm.h"
#include "calibration.h"
#include "camera_reprojection_functor.h"
#include "filter_state.h"
#include "imu_item.h"
#include "imu_buffer.h"
#include "quaternion_tools.h"

Filter::Filter(std::shared_ptr<const Calibration> calibration, std::shared_ptr<const StateInitializer> state_initializer)
: calibration_(calibration), state_initializer_(state_initializer), feature_tracker_(calibration->getNumberOfFeaturesToExtract()) {
}

void Filter::stepInertial(double time, const ImuItem &accel, const ImuItem &gyro) {
    if (!is_initialized_) {
        initialize(time, accel, gyro);
    }
    Eigen::Vector3d acceleration_estimate = computeAccelerationEstimate(accel.getVector());
    Eigen::Vector3d rotation_estimate = computeRotationEstimate(gyro.getVector(), acceleration_estimate);
    
    assert(state().body_state_.get());
    state().body_state_ = BodyState::propagate(
            *(state().body_state_.get()), time, rotation_estimate, acceleration_estimate);
}

void Filter::stepCamera(double time, cv::Mat& frame, const ImuBuffer::iterator& hint_gyro, const ImuBuffer::iterator& hint_accel) {
    if (!is_initialized_) {
        return;
    }
    augment(hint_gyro, hint_accel);
    CameraPose& last_camera_pose = state().poses().back();
    
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
    performUpdate(features_to_rezidualize);
    
    for (std::size_t i = 0; i < features_to_rezidualize.size(); ++i) {
        features_to_rezidualize[i]->drawFeatureTrack(frame, cv::Scalar(0, 0, 255));
    }

    pruneCameraPoses(features_to_rezidualize);
    features_tracked_ = current_features_tracked;
    
    std::cout << state() << std::endl;
}

Eigen::Vector3d Filter::getCurrentPosition() {
    return state().getPositionInGlobalFrame();
}

Eigen::Quaterniond Filter::getCurrentAttitude() {
    return state().getOrientationInGlobalFrame();
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

Eigen::Quaterniond Filter::getBodyToCameraRotation() const {
    return calibration_->getBodyToCameraRotation();
}

Eigen::Vector3d Filter::getPositionOfBodyInCameraFrame() const {
    return state().position_of_body_in_camera_;
}

const FilterState& Filter::state() const {
    return *filter_state_;
}

void Filter::setInitialBodyPositionInCameraFrame(const Eigen::Vector3d& position) {
    initial_body_position_in_camera_frame_ = position;
}

FilterState& Filter::state() {
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
    
    Eigen::Quaterniond attitude = state_initializer_->getOrientation();
    Eigen::Vector3d position = state_initializer_->getPosition();
    Eigen::Vector3d velocity = state_initializer_->getVelocity();
    
    state().body_state_ = std::make_shared<BodyState>(calibration_, time, rotation_estimate, acceleration_estiamte, attitude, position, velocity);
        
    Eigen::Matrix<double, 56, 1> covar_diag;
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
    
    filter_covar_ = covar_diag.asDiagonal();
    
    is_initialized_ = true;
    
    assert(state().body_state_.get());
}

Eigen::Vector3d Filter::computeRotationEstimate(const Eigen::Vector3d &gyro, const Eigen::Vector3d &acceleration_estimate) const {
    Eigen::Matrix3d T_g_inv = state().gyroscope_shape_.inverse();
    Eigen::Matrix3d T_s = state().gyroscope_acceleration_sensitivity_;
    Eigen::Vector3d b_g = state().bias_gyroscope_;
//    std::cout << "Gyro      : [" << gyro.transpose() << "]^T" << std::endl;
//    std::cout << "Bias      : [" << b_g.transpose() << "]^T" << std::endl;
//    std::cout << "Gyro clear: [" << (T_g_inv * (gyro - T_s*acceleration_estimate - b_g)).transpose() << "]^T" << std::endl;
    
    Eigen::Vector3d res = T_g_inv * (gyro - T_s*acceleration_estimate - b_g);
//    assert(res.norm() > 1e-100);
    assert(res.norm() < 1e100);
    
    return res;
}

Eigen::Vector3d Filter::computeAccelerationEstimate(const Eigen::Vector3d &accel) const {
    Eigen::Matrix3d T_a_inv = state().accelerometer_shape_.inverse();
    Eigen::Vector3d b_a = state().bias_accelerometer_;
//        std::cout << "Accel      : [" << accel.transpose() << "]^T" << std::endl;
//        std::cout << "Bias       : [" << b_a.transpose() << "]^T" << std::endl;
//        std::cout << "Accel clear: [" << (accel - b_a).transpose() << "]^T" << std::endl;
    
    
    Eigen::Vector3d res = T_a_inv * (accel - b_a);
    //assert(res.norm() > 1e-100);
    assert(res.norm() < 1e100);
    
    return res;
}

void Filter::augment(const ImuBuffer::iterator& hint_gyro, const ImuBuffer::iterator& hint_accel) {
    CameraPose new_pose(*state().body_state_, hint_gyro, hint_accel);
    state().poses().addNewCameraPose(std::move(new_pose));
    Eigen::MatrixXd new_covar_(filter_covar_.rows() + 9, filter_covar_.cols() + 9);
    Eigen::MatrixXd J_pi = Eigen::MatrixXd::Identity(9, filter_covar_.cols());
    new_covar_.block(0, 0, filter_covar_.rows(), filter_covar_.cols()) = filter_covar_;
    new_covar_.block(filter_covar_.rows(), 0, 9, filter_covar_.cols()) = J_pi * filter_covar_;
    new_covar_.block(0, filter_covar_.cols(), filter_covar_.rows(), 9) = new_covar_.block(filter_covar_.rows(), 0, 9, filter_covar_.cols()).transpose();
    new_covar_.block<9, 9>(filter_covar_.rows(), filter_covar_.cols()) = new_covar_.block(filter_covar_.rows(), 0, 9, filter_covar_.cols())*J_pi.transpose();
    filter_covar_ = new_covar_;
}

void Filter::pruneCameraPoses(const FeatureTracker::feature_track_list& residualized_features) {
    for (std::size_t i = 0; i < residualized_features.size(); ++i) {
        CameraPoseBuffer::iterator it = std::end(state().poses());
        it = std::prev(it);
        
        for (std::size_t j = 0; j < residualized_features[i]->posesTrackedCount(); ++j) {
            it = std::prev(it);
            it->decreaseActiveFeaturesCount(residualized_features[i]->getFeatureId());
        }
    }
    
    std::size_t poses_deleted = 0;
    CameraPoseBuffer& poses = state().poses();
    while (!poses.empty() && poses.front().getActiveFeaturesCount() == 0) {
        poses.deleteOldestCameraPose();
        poses_deleted += 1;
    }
    
    Eigen::MatrixXd new_covar_(filter_covar_.rows() - 9*poses_deleted, filter_covar_.cols() - 9*poses_deleted);
    new_covar_.block<56, 56>(0, 0) = filter_covar_.block<56, 56>(0, 0);
    new_covar_.bottomRows(new_covar_.rows() - 56).leftCols<56>() = filter_covar_.bottomRows(new_covar_.rows() - 56).leftCols<56>();
    new_covar_.topRows<56>().rightCols(new_covar_.cols() - 56) = new_covar_.bottomRows(new_covar_.rows() - 56).leftCols<56>().transpose();
    new_covar_.bottomRows(new_covar_.rows() - 56).rightCols(new_covar_.cols() - 56) = filter_covar_.bottomRows(new_covar_.rows() - 56).rightCols(new_covar_.cols() - 56);
    filter_covar_ = new_covar_;
    
    assert(poses.size() <= calibration_->getMaxCameraPoses());
}

FeatureRezidualizationResult Filter::rezidualizeFeature(const FeatureTrack& feature_track) const {
    CameraPoseBuffer pose_buffer = state().poses();
    std::size_t track_length = feature_track.posesTrackedCount();
    std::size_t poses_in_state = pose_buffer.size();
    FeatureRezidualizationResult result(track_length, poses_in_state);
    
    std::pair<bool, Eigen::Vector3d> global_position_result = triangulateGlobalFeaturePosition(feature_track);
    bool global_position_success = global_position_result.first;
    Eigen::Vector3d global_position = global_position_result.second;
    if (!global_position_success) {
        result.setIsInvalid();
        return result;
    }
    if ((global_position - filter_state_->getPositionInGlobalFrame()).norm() > 1000) {
        std::cout << "Suspicious global position [" << global_position.transpose() << "]^T" << std::endl;
    } else {
        std::cout << "Norm: " << (global_position - filter_state_->getPositionInGlobalFrame()).norm() << std::endl;
    }
    
    for (std::size_t j = 0; j < track_length; ++j) {
        Eigen::Vector2d z = feature_track[j];
        
        CameraPose& pose = pose_buffer[poses_in_state - track_length + j - 1];
        double pose_time = pose.time();
        std::size_t N = frame_rows_;
        std::size_t k = z(1) - N / 2;
        double feature_time = pose_time + state().camera_readout_*k/N;
        
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
        std::cout << "p_B_C " << p_B_C.transpose() << std::endl;
        
        Eigen::Vector3d p_f_C = R_C_B*R_Bj_G*(global_position - p_Bj_G) + p_B_C;
        
        std::cout << "x C_" << j << ": [" << p_f_C.transpose() << "]^T" << std::endl;
        
        if (p_f_C(2) < 0) {
            // Feature is behind camera
            result.setIsInvalid();
            return result;
        }
        Eigen::Vector2d z_hat = cameraProject(p_f_C);
        if ((z - z_hat).norm() > 1000) {
            std::cout << "Residuum " << (z - z_hat).norm() << std::endl;
        }
        result.setPoseRezidual(j, z - z_hat);
        
        Eigen::Matrix<double, 2, 3> J_h = cameraProjectJacobian(p_f_C);
        Eigen::Matrix3d rotation_compound = R_C_B * R_Bj_G;
        Eigen::Matrix<double, 2, 3> M_i_j = J_h * rotation_compound;
        
        Eigen::Matrix<double, 3, 9> H_x_Bj_rhs_part;
        H_x_Bj_rhs_part.block<3, 3>(0, 0) = QuaternionTools::crossMatrix(global_position - p_Bj_G);
        H_x_Bj_rhs_part.block<3, 3>(0, 3) = -1*Eigen::Matrix3d::Identity();
        H_x_Bj_rhs_part.block<3, 3>(0, 6) = (-1.0*k*state().camera_readout_/N)*Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 2, 9> H_x_Bj = M_i_j * H_x_Bj_rhs_part;
        result.setJacobianByCameraPose(j, H_x_Bj);
        
        Eigen::Matrix<double, 2, 3> H_f_ij = M_i_j;
        result.setJacobianByFeaturePosition(j, H_f_ij);
        
        Eigen::Matrix<double, 2, 3> z_by_p_B_C = J_h;
        Eigen::Matrix3d cross_matrix_part;
        cross_matrix_part = QuaternionTools::crossMatrix(R_Bj_G*(global_position - p_Bj_G));
        Eigen::Vector2d z_by_td = J_h*R_C_B*(cross_matrix_part*rotation_estimate - R_Bj_G*v_Bj_G);
        Eigen::Vector2d z_by_tr = k/N*z_by_td;
        
        double u = global_position(0) / global_position(2);
        double v = global_position(1) / global_position(2);
        double r = u*u + v*v;
        double k1 = state().radial_distortion_(0);
        double k2 = state().radial_distortion_(1);
        double k3 = state().radial_distortion_(2);
        double t1 = state().tangential_distortion_(0);
        double t2 = state().tangential_distortion_(1);
        double fx = state().focal_point_(0);
        double fy = state().focal_point_(1);
        double dr = 1.0 + k1*r + k2*r*r + k3*std::pow(r, 3.0);
        Eigen::Vector2d dt;
        dt(0) = 2.0*u*v*t1 + (r + 2.0*u*u)*t2;
        dt(1) = 2.0*u*v*t2 + (r + 2.0*v*v)*t1;
        
        Eigen::Vector2d h_by_fx;
        h_by_fx(0) = dr*u + dt(0);
        h_by_fx(1) = 0.0;
        Eigen::Vector2d h_by_fy;
        h_by_fy(0) = 0.0;
        h_by_fy(1) = dr*v + dt(1);
        Eigen::Vector2d h_by_ox = Eigen::Vector2d::UnitX();
        Eigen::Vector2d h_by_oy = Eigen::Vector2d::UnitY();
        Eigen::Vector2d h_by_k1;
        h_by_k1(0) = fx*u*r;
        h_by_k1(1) = fy*v*r;
        Eigen::Vector2d h_by_k2 = r * h_by_k1;
        Eigen::Vector2d h_by_k3 = r * h_by_k2;
        Eigen::Vector2d h_by_t1;
        h_by_t1(0) = fx*(2.0*u*v);
        h_by_t1(1) = fy*(r + 2.0*v*v);
        Eigen::Vector2d h_by_t2;
        h_by_t2(0) = fx*(r + 2.0*u*u);
        h_by_t2(1) = fy*(2*u*v);
        
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
        
        result.setJacobianByCameraParameters(j, H_c);
    }

    return result;
}

void Filter::performUpdate(const FeatureTracker::feature_track_list& features_to_rezidualize) {
    std::vector<Eigen::VectorXd> r_list;
    std::vector<Eigen::MatrixXd> H_list;
    std::size_t H_rows = 0;
    
    std::size_t num_poses = state().poses().size();
    std::vector<Eigen::Vector3d> global_positions;
    
    int hist[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (const std::shared_ptr<FeatureTrack>& feature : features_to_rezidualize) {
        if (feature->posesTrackedCount() < 10) {
            hist[feature->posesTrackedCount()] += 1;
        }
    }
    for (std::size_t i = 0; i < 10; ++i) {
        std::cout << i << " " << hist[i] << " | ";
    }
    std::cout << std::endl;
    
    for (const std::shared_ptr<FeatureTrack>& feature : features_to_rezidualize) {
        if (feature->posesTrackedCount() <= 3) {
            continue;
        }
        
        FeatureRezidualizationResult rezidualization_result = rezidualizeFeature(*feature);
        if (!rezidualization_result) {
            std::cout << "Skipping feature due to unsuccessful global position estimation." << std::endl;
            continue;
        }
    
        const Eigen::Matrix<double, Eigen::Dynamic, 3>& H_f = rezidualization_result.getJacobianByFeaturePosition();
        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(H_f.transpose());
        Eigen::MatrixXd A_t = lu_decomp.kernel().transpose();
        
//        auto blahblah = A_t*H_f;
//        std::cout.precision(12);
//        std::cout << "Max vole " << blahblah.maxCoeff() << std::endl;
//        assert(blahblah.isZero(1e-6));
        
        Eigen::VectorXd r_0_i = A_t*rezidualization_result.getReziduals();
        Eigen::MatrixXd H_0_i = A_t*rezidualization_result.getJacobianByState();
        
        // @todo: Implement Sigma matrix
        if (true || gatingTest(r_0_i, H_0_i)) {
            
            if (r_0_i.maxCoeff() > 1000) {
                std::cout << "Max r_0_i coeff " << r_0_i.maxCoeff() << std::endl;
                continue;
            }
            
            r_list.push_back(r_0_i);
            H_list.push_back(H_0_i);
            H_rows += H_0_i.rows();
        } else {
            std::cout << "Feature is outlier." << std::endl;
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
    {
        Eigen::VectorXd r(H_rows);
        Eigen::MatrixXd H(H_rows, 56+9*num_poses);
        std::size_t row_processed = 0;
        for (std::size_t i = 0; i < H_list.size(); ++i) {
            r.segment(row_processed, r_list[i].rows()) = r_list[i];
            H.block(row_processed, 0, H_list[i].rows(), H_list[i].cols()) = H_list[i];
            row_processed += r_list[i].rows();
        }
        
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
    }
    
    updateState(T_H, r_q);
}

bool Filter::gatingTest(const Eigen::VectorXd& r_0_i, const Eigen::MatrixXd H_0_i) {
    auto inner = H_0_i*filter_covar_*H_0_i.transpose();
    auto rows = inner.rows();
    auto cols = inner.cols();
    assert(rows == cols);
    Eigen::FullPivLU<Eigen::MatrixXd> full_piv_lu(inner);
    if (!full_piv_lu.isInvertible()) {
        
        return false;
    }
    auto inner_inverted = inner.inverse();
    Eigen::MatrixXd gamma_i = r_0_i.transpose()*inner_inverted*r_0_i;
    bool test_passed = gamma_i(0, 0) < CHI_SQUARED_PPM_VALUES[r_0_i.rows()];
    if (!test_passed) {
        std::cout << "Test failed with gamma_i: " << gamma_i(0, 0) << " threshold: " << CHI_SQUARED_PPM_VALUES[r_0_i.cols()] << std::endl;
    }
    return test_passed;
}

void Filter::updateState(const Eigen::MatrixXd& T_H, const Eigen::VectorXd& r_q) {
    double sigma_squared = 1e-2;
    Eigen::MatrixXd R_q = sigma_squared * Eigen::MatrixXd::Identity(T_H.rows(), T_H.rows());
    // Kalman gain
    Eigen::MatrixXd K = filter_covar_*T_H.transpose()*(T_H*filter_covar_*T_H.transpose() + R_q).inverse();
    Eigen::MatrixXd I_beta = Eigen::MatrixXd::Identity(T_H.cols(), T_H.cols());
    Eigen::MatrixXd covar_prop_part = I_beta - K*T_H;
    filter_covar_ = covar_prop_part*filter_covar_*covar_prop_part.transpose() + K*R_q*K.transpose();
    Eigen::VectorXd delta_x = K*r_q;
    filter_state_->updateWithStateDelta(delta_x);
}

Eigen::Vector3d Filter::initialGuessFeaturePosition(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, const Eigen::Matrix3d& R_C1_C0, const Eigen::Vector3d& p_C1_C0, InitialGuessMethod method) const {
    Eigen::Vector3d v1;
    v1 << z0(0), z0(1), 1.0;
    Eigen::Vector3d v2;
    v2 << z1(0), z1(1), 1.0;
    
    v1.normalize();
    v2.normalize();
    
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> a;
    a.resize(3, 2);
    a.block<3, 1>(0, 0) = v1;
    a.block<3, 1>(0, 1) = -1*R_C1_C0*v2;
    Eigen::Vector3d b = p_C1_C0;
    
    Eigen::Vector2d x;
    
    if (method == InitialGuessMethod::SVD) {
        x = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    } else if (method == InitialGuessMethod::QR) {
        x = a.colPivHouseholderQr().solve(b);
    } else if (method == InitialGuessMethod::normal) {
        x = (a.transpose() * a).ldlt().solve(a.transpose() * b);
    } else {
        throw std::invalid_argument("ReprojectionOptimizer::initialGuess method must be either SVD, QR or normal");
    }
    
    return x(0) * v1;
}

std::pair<bool, Eigen::Vector3d> Filter::triangulateGlobalFeaturePosition(const FeatureTrack &feature_track) const {
    std::size_t n = feature_track.posesTrackedCount();
    
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector2d> measurements;
    
    const CameraPoseBuffer& poses = state().poses();
    assert(n <= poses.size() - 1);
    CameraPoseBuffer::const_iterator it_c0 = std::next(poses.begin(), poses.size() - (n+1));
    CameraPoseBuffer::const_iterator it_end = std::prev(std::end(poses));
    for (auto it = it_c0; it != it_end; ++it) {
        const CameraPose& c0 = *it_c0;
        const CameraPose& ci = *it;
        
        Eigen::Quaterniond q_Ci_C0 = c0.getRotationToOtherPose(ci, *this);
        Eigen::Vector3d p_C0_Ci = ci.getPositionOfAnotherPose(c0, *this);
        Eigen::Vector2d z_i = feature_track[it - it_c0];
        
        rotations.push_back(q_Ci_C0.toRotationMatrix());
        positions.push_back(p_C0_Ci);
        measurements.push_back(z_i);
    }
    
    // Initial guess
    const Eigen::Vector2d& z0 = (feature_track[0].array() - filter_state_->optical_center_.array()) / filter_state_->focal_point_.array();
    const Eigen::Vector2d& z1 = (feature_track[1].array() - filter_state_->optical_center_.array()) / filter_state_->focal_point_.array();
    const Eigen::Quaterniond q_C1_C0 = (it_c0)->getRotationToOtherPose(*(it_c0 + 1), *this);
    const Eigen::Vector3d& p_C1_C0 = (it_c0)->getPositionOfAnotherPose(*(it_c0 + 1), *this);
    Eigen::Vector3d initial_guess = initialGuessFeaturePosition(z0, z1, q_C1_C0.toRotationMatrix(), p_C1_C0, InitialGuessMethod::SVD);
    Eigen::VectorXd x = initial_guess;
    x(2) = 1.0;
    x /= initial_guess(2);
    
    // Solve
    CameraReprojectionFunctor functor(rotations, positions, measurements, *this);
    Eigen::NumericalDiff<CameraReprojectionFunctor> num_diff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CameraReprojectionFunctor>> lm(num_diff);
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(x);
    
    std::string msg;
    switch (status) {
        case Eigen::LevenbergMarquardtSpace::Status::NotStarted:
            msg = "NotStarted";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::Running:
            msg = "Running";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::ImproperInputParameters:
            msg = "ImproperInputParameters";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::RelativeReductionTooSmall:
            msg = "RelativeReductionTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::RelativeErrorTooSmall:
            msg = "RelativeErrorTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::RelativeErrorAndReductionTooSmall:
            msg = "RelativeErrorAndReductionTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::CosinusTooSmall:
            msg = "CosinusTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::TooManyFunctionEvaluation:
            msg = "TooManyFunctionEvaluation";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::FtolTooSmall:
            msg = "FtolTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::XtolTooSmall:
            msg = "XtolTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::GtolTooSmall:
            msg = "GtolTooSmall";
            break;
        case Eigen::LevenbergMarquardtSpace::Status::UserAsked:
            msg = "UserAsked";
            break;
            
        default:
            msg = "UNKNOWN";
            break;
    }
    std::string status_msg;
    switch (lm.info()) {
        case Eigen::ComputationInfo::Success:
            status_msg = "Success";
            break;
        case Eigen::ComputationInfo::NumericalIssue:
            status_msg = "NumericalIssue";
            break;
        case Eigen::ComputationInfo::NoConvergence:
            status_msg = "NoConvergence";
            break;
        case Eigen::ComputationInfo::InvalidInput:
            status_msg = "InvalidInput";
            break;
            
        default:
            break;
    }
//    std::cout << "Optimization ended after " << lm.iterations() << " iteration(s) with status " << status_msg << " (" << msg << ")" << std::endl;
    
    Eigen::VectorXd fvec;
    fvec.resize(functor.values());
    functor(x, fvec);
//    std::cout << "Residualization maxCoeff " << fvec.maxCoeff() << std::endl;
//    if (fvec.maxCoeff() > 1000) {
//        std::cout << "Residualization maxCoeff too big " << fvec.maxCoeff() << std::endl;
//    }
    
    // Global position estimate using from inverse depth parametrization of feature position in first camera pose that observed it.
    Eigen::Quaterniond q_G_C0 = it_c0->getCameraOrientationInGlobalFrame(*this).conjugate();
    Eigen::Vector3d p_C0_G = it_c0->getCameraPositionInGlobalFrame(*this);
    Eigen::Vector3d param;
    param << x(0), x(1), 1.0;
    double rho = x(2);
    
    Eigen::Vector3d global_position = 1.0/rho * q_G_C0.toRotationMatrix()*param + p_C0_G;
    if (std::isnan(global_position.norm())) {
        throw std::runtime_error("Global feature position is NaN");
    }
    
    return std::make_pair(fvec.maxCoeff() <= 1000, global_position);
}

Eigen::Vector2d Filter::cameraProject(const Eigen::Vector3d& p) const {
    double u = p(0) / p(2);
    double v = p(1) / p(2);
    Eigen::Vector2d uv_vec;
    uv_vec << u, v;
    double r = u*u + v*v;
    double k_1 = state().radial_distortion_[0];
    double k_2 = state().radial_distortion_[1];
    double k_3 = state().radial_distortion_[2];
    double t_1 = state().tangential_distortion_[0];
    double t_2 = state().tangential_distortion_[1];
    double d_r = 1 + k_1*r + k_2*r*r + k_3*r*r*r;
    Eigen::Vector2d d_t;
    d_t <<
        2*u*v*t_1 + (r + 2*u*u)*t_2,
        2*u*v*t_2 + (r + 2*v*v)*t_1;
    return state().optical_center_ + state().focal_point_.asDiagonal()*(d_r*uv_vec + d_t);
}

Eigen::Matrix<double, 2, 3> Filter::cameraProjectJacobian(const Eigen::Vector3d& p) const {
    double x = p(0);
    double y = p(1);
    double z = p(2);
    double u = x / z;
    double v = y / z;
    double r = u*u + v*v;
    double k1 = state().radial_distortion_(0);
    double k2 = state().radial_distortion_(1);
    double k3 = state().radial_distortion_(2);
    double t1 = state().tangential_distortion_(0);
    double t2 = state().tangential_distortion_(1);
    double dr = 1.0 + k1*r + k2*r*r + k3*std::pow(r, 3.0);
    
    Eigen::RowVector3d uv_by_xyz;
    uv_by_xyz << y/(z*z), x/(z*z), -2.0*x*y/std::pow(z, 3.0);
    
    Eigen::RowVector3d u_by_xyz;
    u_by_xyz << 1.0/z, 0.0, -1.0*x/(z*z);
    
    Eigen::RowVector3d v_by_xyz;
    v_by_xyz << 0.0, 1.0/z, -1.0*y/(z*z);
    
    double x2_y2 = x*x + y*y;
    
    Eigen::RowVector3d r_by_xyz;
    r_by_xyz << 2.0*x/(z*z), 2.0*y/(z*z), -2.0/std::pow(z, 3.0)*(x2_y2);
    
    Eigen::RowVector3d r_squared_by_xyz;
    r_squared_by_xyz << 4.0*x*(x2_y2)/std::pow(z, 4.0), 4.0*y*x2_y2/std::pow(z, 4.0), -4.0*x2_y2*x2_y2/std::pow(z, 5.0);
    
    Eigen::RowVector3d r_cubed_by_xyz;
    r_cubed_by_xyz << 6.0*x*x2_y2*x2_y2/std::pow(z, 6.0), 6.0*y*x2_y2*x2_y2/std::pow(z, 6.0), -6.0*std::pow(x2_y2, 3.0)/std::pow(z, 7.0);
    
    Eigen::RowVector3d dr_by_xyz = Eigen::RowVector3d::Constant(1.0) + k1*r_by_xyz + k2*r_squared_by_xyz + k3*r_cubed_by_xyz;
    
    Eigen::Matrix<double, 2, 3> dt_by_xyz;
    dt_by_xyz.block<1, 3>(0, 0) = 2.0*t1*uv_by_xyz + t2*r_by_xyz + 4.0*t2*u*u_by_xyz;
    dt_by_xyz.block<1, 3>(1, 0) = 2.0*t2*uv_by_xyz + t1*r_by_xyz + 4.0*t1*v*v_by_xyz;
    
    Eigen::Matrix2d lhs = state().focal_point_.asDiagonal();
    
    Eigen::Matrix<double, 2, 3> rhs;
    rhs(0, 0) = dr/z + dr_by_xyz(0)*u + dt_by_xyz(0);
    rhs(0, 1) = dr_by_xyz(1)*u + dt_by_xyz(1);
    rhs(0, 2) = -1.0*dr/z*u + dr_by_xyz(2)*u + dt_by_xyz(2);
    rhs(1, 0) = dr_by_xyz(0)*v + dt_by_xyz(0);
    rhs(1, 1) = dr/z + dr_by_xyz(1)*v + dt_by_xyz(1);
    rhs(1, 2) = -1.0*dr/z*v + dr_by_xyz(2)*v + dt_by_xyz(2);
    
    return lhs*rhs;
}



