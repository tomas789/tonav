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

#include "calibration.h"
#include "camera_reprojection_functor.h"
#include "filter_state.h"
#include "imu_item.h"

Filter::Filter(std::shared_ptr<const Calibration> calibration)
: calibration_(calibration), debug_output_("/Users/tomaskrejci/tonav_debug.log", std::ios::out), filter_state_(calibration), feature_tracker_(calibration->getNumberOfFeaturesToExtract()) {
    
}

void Filter::stepInertial(double time, const ImuItem &accel, const ImuItem &gyro) {
    if (!is_initialized_) {
        initialize(time, accel, gyro);
    }
    Eigen::Vector3d acceleration_estimate = computeAccelerationEstimate(accel.getVector());
    Eigen::Vector3d rotation_estimate = computeRotationEstimate(gyro.getVector(), acceleration_estimate);
    
    assert(filter_state_.body_state_.get());
    filter_state_.body_state_ = BodyState::propagate(
            *(filter_state_.body_state_.get()), time, rotation_estimate, acceleration_estimate);
}

void Filter::stepCamera(double time, cv::Mat& frame) {
    if (!is_initialized_) {
        return;
    }
    augment();
    CameraPose& last_camera_pose = filter_state_.poses().back();
    
    frame_rows_ = frame.rows;
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
        assert(features_tracked_[i]->posesTrackedCount() <= calibration_->getMaxCameraPoses() + 1 || features_tracked_[i]->wasUsedForResidualization());
        if (features_tracked_[i]->wasUsedForResidualization()) {
            continue;
        }
        if (features_tracked_[i]->isOutOfView()) {
            features_tracked_[i]->setWasUsedForResidualization();
            features_to_residualize.push_back(features_tracked_[i]);
        } else if (features_tracked_[i]->posesTrackedCount() == calibration_->getMaxCameraPoses() + 1) {
            features_tracked_[i]->setWasUsedForResidualization();
            features_tracked_[i]->revertLastPosition();
            last_camera_pose.decreaseActiveFeaturesCount(features_tracked_[i]->getFeatureId());
            features_to_residualize.push_back(features_tracked_[i]);
        }
    }
    
    std::size_t num_poses = filter_state_.poses().size();
    const CameraPoseBuffer& pose_buffer = filter_state_.poses();
    std::vector<Eigen::Vector3d> global_positions;
    for (const std::shared_ptr<FeatureTrack>& feature : features_to_residualize) {
        if (feature->posesTrackedCount() <= 3) {
            continue;
        }
        
        std::pair<Eigen::Vector3d, Eigen::VectorXd> global_position_estimation_result;
        global_position_estimation_result = triangulateGlobalFeaturePosition(*feature);
        Eigen::Vector3d global_position = global_position_estimation_result.first;
        Eigen::VectorXd reziduals = global_position_estimation_result.second;
        
        std::size_t track_length = feature->posesTrackedCount();
        
        Eigen::VectorXd r_0;
        r_0.resize(2*track_length);
        Eigen::MatrixXd H_f;
        H_f.resize(2*track_length, 3);
        Eigen::MatrixXd H_x;
        H_x.resize(2*track_length, 56+9*num_poses);
        
        for (std::size_t i = 0; i < track_length; ++i) {
            Eigen::Vector2d z = (*feature)[i];
            
            const CameraPose& pose = pose_buffer[num_poses - track_length + i - 1];
            double pose_time = pose.time();
            std::size_t k = z(1) - frame_rows_ / 2;
            double feature_time = pose_time + filter_state_.camera_readout_*k/frame_rows_;
            
            ImuItem accel_item = accelerometer_interpolation_callback_(pose_time);
            ImuItem gyro_item = gyroscope_interpolation_callback_(pose_time);
            Eigen::Vector3d acceleration_estimate = computeAccelerationEstimate(accel_item.getVector());
            Eigen::Vector3d rotation_estimate = computeRotationEstimate(gyro_item.getVector(), acceleration_estimate);
            std::shared_ptr<BodyState> body_state_for_feature = BodyState::propagate(pose.getBodyState(), feature_time, rotation_estimate, acceleration_estimate);
            
            Eigen::Matrix3d R_C_B = getBodyToCameraRotation().toRotationMatrix();
            Eigen::Matrix3d R_Bj_G = body_state_for_feature->getOrientationInGlobalFrame().toRotationMatrix();
            Eigen::Vector3d p_Bj_G = body_state_for_feature->getPositionInGlobalFrame();
            Eigen::Vector3d p_B_C = getPositionOfBodyInCameraFrame();
            
            Eigen::Vector3d p_f_C = R_C_B*R_Bj_G*(global_position - p_Bj_G) + p_B_C;
        }
        
        
        
        
        
        
        global_positions.push_back(global_position);
        debug_output_ << "F," << filter_state_.time() << "," << global_position(0) << "," << global_position(1) << "," << global_position(2) << std::endl;
    }
    debug_output_ << "P," << filter_state_.time() << "," << getCurrentPosition()(0) << "," << getCurrentPosition()(1) << "," << getCurrentPosition()(2) << std::endl;
    debug_output_.flush();
    
    for (std::size_t i = 0; i < features_to_residualize.size(); ++i) {
        features_to_residualize[i]->drawFeatureTrack(frame, cv::Scalar(0, 0, 255));
        
        
    }
    
    
    
    pruneCameraPoses(features_to_residualize);
    features_tracked_ = current_features_tracked;
    
    std::cout << filter_state_ << std::endl;
}

Eigen::Vector3d Filter::getCurrentPosition() {
    return filter_state_.getPositionInGlobalFrame();
}

Eigen::Quaterniond Filter::getCurrentAttitude() {
    return filter_state_.getOrientationInGlobalFrame();
}

double Filter::getImageCaptureTime(double arrive_time) {
    return arrive_time + filter_state_.camera_delay_;
}

double Filter::getImageFirstLineCaptureTime(double arrive_time) {
    return arrive_time + filter_state_.camera_delay_ - std::abs(filter_state_.camera_readout_) / 2.0;
}

double Filter::getImageLastLineCaptureTime(double arrive_time) {
    return arrive_time + filter_state_.camera_delay_ + std::abs(filter_state_.camera_readout_) / 2.0;
}

bool Filter::isInitialized() const {
    return is_initialized_;
}

double Filter::time() const {
    if (!isInitialized()) {
        return NAN;
    }
    return filter_state_.time();
}

Eigen::Quaterniond Filter::getBodyToCameraRotation() const {
    return calibration_->getBodyToCameraRotation();
}

Eigen::Vector3d Filter::getPositionOfBodyInCameraFrame() const {
    return filter_state_.position_of_body_in_camera_;
}

const FilterState& Filter::state() const {
    return filter_state_;
}

void Filter::setInitialOrientation(const Eigen::Quaterniond& orientation) {
    initial_orientation_ = orientation;
}

void Filter::setInitialPosition(const Eigen::Vector3d& position) {
    initial_position_ = position;
}

void Filter::setInitialBodyPositionInCameraFrame(const Eigen::Vector3d& position) {
    initial_body_position_in_camera_frame_ = position;
}

void Filter::setGyroscopeInterpolationCallback(std::function<ImuItem(double)> callback) {
    gyroscope_interpolation_callback_ = callback;
}

void Filter::setAccelerometerInterpolationCallback(std::function<ImuItem(double)> callback) {
    accelerometer_interpolation_callback_ = callback;
}

void Filter::initialize(double time, const ImuItem &accel, const ImuItem &gyro) {
    FilterState state(calibration_);
    
    state.bias_gyroscope_ = calibration_->getGyroscopeBias();
    state.bias_accelerometer_ = calibration_->getAccelerometerBias();
    state.gyroscope_shape_ = calibration_->getGyroscopeShapeMatrix();
    state.gyroscope_acceleration_sensitivity_ = calibration_->getGyroscopeAccelerationSensitivityMatrix();
    state.accelerometer_shape_ = calibration_->getAccelerometerShapeMatrix();
    state.position_of_body_in_camera_ = initial_body_position_in_camera_frame_;
    state.focal_point_ = calibration_->getCameraFocalPoint();
    state.optical_center_ = calibration_->getCameraOpticalCenter();
    state.radial_distortion_ = calibration_->getCameraRadialDistortionParams();
    state.tangential_distortion_ = calibration_->getCameraTangentialDistortionParams();
    state.camera_delay_ = calibration_->getCameraDelayTime();
    state.camera_readout_ = calibration_->getCameraReadoutTime();

    filter_state_ = state;
    
    Eigen::Vector3d acceleration_estiamte = computeAccelerationEstimate(accel.getVector());
    Eigen::Vector3d rotation_estimate = computeRotationEstimate(gyro.getVector(), acceleration_estiamte);
    
    Eigen::Quaterniond attitude = initial_orientation_;
    Eigen::Vector3d position = initial_position_;
    
    Eigen::Vector3d velocity;
    velocity << 4.40446529345980, -0.02522437364702, 0.01667368746900;
    // Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    
    filter_state_.body_state_ = std::make_shared<BodyState>(calibration_, time, rotation_estimate, acceleration_estiamte, attitude, position, velocity);
    
    is_initialized_ = true;
    
    assert(filter_state_.body_state_.get());
}

Eigen::Vector3d Filter::computeRotationEstimate(const Eigen::Vector3d &gyro, const Eigen::Vector3d &acceleration_estimate) const {
    Eigen::Matrix3d T_g_inv = filter_state_.gyroscope_shape_.inverse();
    Eigen::Matrix3d T_s = filter_state_.gyroscope_acceleration_sensitivity_;
    Eigen::Vector3d b_g = filter_state_.bias_gyroscope_;
//    std::cout << "Gyro      : [" << gyro.transpose() << "]^T" << std::endl;
//    std::cout << "Bias      : [" << b_g.transpose() << "]^T" << std::endl;
//    std::cout << "Gyro clear: [" << (T_g_inv * (gyro - T_s*acceleration_estimate - b_g)).transpose() << "]^T" << std::endl;
    return T_g_inv * (gyro - T_s*acceleration_estimate - b_g);
}

Eigen::Vector3d Filter::computeAccelerationEstimate(const Eigen::Vector3d &accel) const {
    Eigen::Matrix3d T_a_inv = filter_state_.accelerometer_shape_.inverse();
    Eigen::Vector3d b_a = filter_state_.bias_accelerometer_;
//        std::cout << "Accel      : [" << accel.transpose() << "]^T" << std::endl;
//        std::cout << "Bias       : [" << b_a.transpose() << "]^T" << std::endl;
//        std::cout << "Accel clear: [" << (accel - b_a).transpose() << "]^T" << std::endl;
    
    return T_a_inv * (accel - b_a);
}

void Filter::augment() {
    filter_state_.poses().addNewCameraPose(*filter_state_.body_state_);
//    CameraPose pose;
//    BodyState& body_state = filter_state_.getBodyStateRef();
//    pose.getRotationForBodyPoseBlock() = body_state.getRotationBlock();
//    pose.getPositionForBodyPoseBlock() = body_state.getPositionBlock();
//    pose.getVelocityForBodyPoseBlock() = body_state.getVelocityBlock();
//    filter_state_.poses().addNewCameraPose(pose);
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
    
    assert(poses.size() <= calibration_->getMaxCameraPoses());
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

std::pair<Eigen::Vector3d, Eigen::VectorXd> Filter::triangulateGlobalFeaturePosition(const FeatureTrack &feature_track) {
    std::size_t n = feature_track.posesTrackedCount();
    
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector2d> measurements;
    
    const CameraPoseBuffer& poses = filter_state_.poses();
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
    const Eigen::Vector2d& z0 = feature_track[0];
    const Eigen::Vector2d& z1 = feature_track[1];
    const Eigen::Quaterniond q_C1_C0 = it_c0->getRotationToOtherPose(*(it_c0 + 1), *this);
    const Eigen::Vector3d& p_C1_C0 = (it_c0 + 1)->getPositionOfAnotherPose(*it_c0, *this);
    Eigen::Vector3d initial_guess = initialGuessFeaturePosition(z0, z1, q_C1_C0.toRotationMatrix(), p_C1_C0, InitialGuessMethod::SVD);
    Eigen::VectorXd x = initial_guess / initial_guess(2);
    
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
    //std::cout << "Optimization ended after " << lm.iterations() << " iteration(s) with status " << status_msg << " (" << msg << ")" << std::endl;
    
    // Global position estimate using from inverse depth parametrization of feature position in first camera pose that observed it.
    Eigen::Quaterniond q_G_C0 = it_c0->getCameraOrientationInGlobalFrame(*this).conjugate();
    Eigen::Vector3d p_C0_G = it_c0->getBodyPositionInGlobalFrame();
    Eigen::Vector3d param;
    param << x(0), x(1), 1.0;
    double rho = x(2);
    
    Eigen::Vector3d global_position = 1.0/rho * q_G_C0.toRotationMatrix()*param + p_C0_G;
    if (std::isnan(global_position.norm())) {
        throw std::runtime_error("Global feature position is NaN");
    }
    
    Eigen::VectorXd reziduals;
    reziduals.resize(functor.values());
    functor(x, reziduals);
    
    return std::make_pair(global_position, reziduals);
}

Eigen::Vector2d Filter::cameraProject(const Eigen::Vector3d& p) const {
    double u = p(0) / p(2);
    double v = p(1) / p(2);
    Eigen::Vector2d uv_vec;
    uv_vec << u, v;
    double r = u*u + v*v;
    double k_1 = filter_state_.radial_distortion_[0];
    double k_2 = filter_state_.radial_distortion_[1];
    double k_3 = filter_state_.radial_distortion_[2];
    double t_1 = filter_state_.tangential_distortion_[0];
    double t_2 = filter_state_.tangential_distortion_[1];
    double d_r = 1 + k_1*r + k_2*r*r + k_3*r*r*r;
    Eigen::Vector2d d_t;
    d_t <<
        2*u*v*t_1 + (r + 2*u*u)*t_2,
        2*u*v*t_2 + (r + 2*v*v)*t_1;
    return filter_state_.optical_center_ + filter_state_.focal_point_.asDiagonal()*(d_r*uv_vec + d_t);
}



