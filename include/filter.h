//
// Created by Tomas Krejci on 5/3/16.
//

#ifndef TONAV_FILTER_H
#define TONAV_FILTER_H

#include <Eigen/Dense>
#include <fstream>
#include <functional>
#include <list>

#include "imu_buffer.h"
#include "calibration.h"
#include "filter_state.h"
#include "feature_rezidualization_result.h"
#include "feature_tracker.h"
#include "state_initializer.h"

class ImuItem;
class CameraReprojectionFunctor;

namespace cv {
    class Mat;
}

enum InitialGuessMethod {
    SVD, QR, normal
};

/**
 * @brief Implementation of MSCKF.
 *
 * This is core functionality of Tonav. All methods must be called with caution. It assumes
 * correct order of update steps.
 *
 * Don't use this class directly. Use Tonav or TonavRos instead
 */
class Filter {
public:
    friend class CameraReprojectionFunctor;
    
    Filter(std::shared_ptr<const Calibration> calibration, std::shared_ptr<const StateInitializer> state_initializer);

    /**
     * @brief Propagate filter using accelerometer and gyroscope measurements
     *
     * @param time Time at which measurements are valid.
     * @param accel Accelerometer measurement.
     * @param gyro Gyroscope measurement.
     */
    void stepInertial(double time, const ImuItem& accel, const ImuItem& gyro);
    
    /**
     * @brief Update filter using camera image.
     *
     * @param time Arrival time of camera image.
     * @param frame Image captured from camera.
     */
    void stepCamera(double time, cv::Mat& frame, const ImuBuffer::iterator& hint_gyro, const ImuBuffer::iterator& hint_accel);

    /** @brief Get current estimated position */
    Eigen::Vector3d getCurrentPosition();

    /**
     * @brief Get current estimated attitude
     *
     * Keep in mind that it returns quaternion as defined by Eigen library. It uses Hamilton's notation of quaternion.
     */
    Eigen::Quaterniond getCurrentAttitude();
    
    /**
     * @brief Calculate \f$\hat{t} = t + \hat{t}_d\f$
     *
     * This method calculates image capture time from time when image arrived.
     *
     * @param arrive_time Time when image arrived
     * @return Estimated image capture time
     */
    double getImageCaptureTime(double arrive_time);
    
    /**
     * @brief Calculate \f$\hat{t} = t + \hat{t}_d - |\hat{t_r}/2|\f$
     */
    double getImageFirstLineCaptureTime(double arrive_time);
    
    /**
     * @brief Calculate \f$\hat{t} = t + \hat{t}_d + |\hat{t_r}/2|\f$
     */
    double getImageLastLineCaptureTime(double arrive_time);
    
    /**
     * @brief Check if filter is initialized.
     *
     * Initialization happens after first `void Filter::stepInertial(double time, const ImuItem& accel, const ImuItem& gyro)` call.
     */
    bool isInitialized() const;
    
    double time() const;
    
    Eigen::Quaterniond getBodyToCameraRotation() const;
    
    Eigen::Vector3d getPositionOfBodyInCameraFrame() const;
    
    /**
     * @brief Get a reference to currently valid filter state.
     */
    const FilterState& state() const;
    
    void setInitialBodyPositionInCameraFrame(const Eigen::Vector3d& position);
protected:
    std::shared_ptr<const Calibration> calibration_;
    std::shared_ptr<const StateInitializer> state_initializer_;
    
    Eigen::Vector3d initial_body_position_in_camera_frame_ = Eigen::Vector3d::Zero();

    /** @brief Initialization status. For delayed initialization. */
    bool is_initialized_ = false;

    /** @brief Filter state \f$ \mathbf{x}_k \f$ */
    std::shared_ptr<FilterState> filter_state_;

    /** @brief Filter covariance matrix \f$ \boldsymbol{\Sigma}_k \f$ */
    Eigen::MatrixXd filter_covar_;

    FeatureTracker feature_tracker_;

    FeatureTracker::feature_track_list features_tracked_;
    
    /** @brief Number of rows in last camera image */
    std::size_t frame_rows_;
    
    /**
     * @brief Get a reference to currently valid filter state.
     *
     * This method is here to make it protected.
     */
    FilterState& state();

    /**
     * @brief Perform delayed initialization.
     *
     * Set all filter parameters from calibration file.
     *
     * I call it delayed initialization because it is performed at the time of first imu step. Not as soon
     * as class instance is created.
     */
    void initialize(double time, const ImuItem& accel, const ImuItem& gyro);

    /**
     * @brief Calculate rotation estimate.
     *
     * It returns best estimate for true rotation rate. It is cleared using estimated gyroscope bias, gyroscope
     * acceleration sensitivity estimate and estimated gyroscope shape matrix.
     *
     * @return \f$ \prescript{B}{}{\boldsymbol{\hat{\omega}}}(t) \f$
     */
    Eigen::Vector3d computeRotationEstimate(const Eigen::Vector3d& gyro, const Eigen::Vector3d& acceleration_estimate) const;

    /**
     * @brief Calculate acceleration estimate.
     *
     * It returns best estimate for true linear acceleration. It is cleared using estimated accelerometer bias and
     * estimated accelerometer shape matrix.
     *
     * @return \f$ \prescript{B}{}{\boldsymbol{\hat{\omega}}}(t) \f$
     */
    Eigen::Vector3d computeAccelerationEstimate(const Eigen::Vector3d& accel) const;

    /**
     * @brief Augment filter state.
     *
     * This is called right before filter update step. It basically does two things. Appends current body state to
     * camera pose buffer (they are not translated into camera frame but kept in body frame instead) and augments
     * filter covariance matrix.
     *
     * @todo Implement this
     */
    void augment(const ImuBuffer::iterator& hint_gyro, const ImuBuffer::iterator& hint_accel);

    /**
     * @brief Remove unused camera poses from filter state.
     *
     * Unused camera poses are those containing no currently tracked feature. Note that camera poses itself does not
     * store tracked features. It is done in `FeatureTracker` class.
     */
    void pruneCameraPoses(const FeatureTracker::feature_track_list& residualized_features);
    
    FeatureRezidualizationResult rezidualizeFeature(const FeatureTrack& feature_track) const;
    
    void performUpdate(const FeatureTracker::feature_track_list& features_to_rezidualize);
    
    bool gatingTest(const Eigen::VectorXd& r_0_i, const Eigen::MatrixXd H_0_i);
    
    void updateState(const Eigen::MatrixXd& T_H, const Eigen::VectorXd& r_q);
    
    /**
     * @brief Initial guess of global feature position from its two measurements.
     */
    Eigen::Vector3d initialGuessFeaturePosition(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, const Eigen::Matrix3d& R_C1_C0, const Eigen::Vector3d& p_C1_C0, InitialGuessMethod method) const;

    /**
     * @brief Calculate \f$ {}^{G}\mathbf{p}_{\mathbf{f}_i} \f$
     *
     * This estimates feature position in global frame. It uses Gauss-Newton algorithm (or Levenberg-Marquardt
     * algorithm) to achieve this. Feature location is described using inverse depth parametrization of feature in
     * first camera pose that captured it.
     *
     * @return Estimate for \f$ {}^{G}\mathbf{p}_{\mathbf{f}_i} \f$.
     */
    std::pair<bool, Eigen::Vector3d> triangulateGlobalFeaturePosition(const FeatureTrack& feature_track) const;
    
    /**
     * @brief Transform inverse depth parametrized estimate of feature's 
     *        position in \f$C_0\f$ to \f$C_i\f$.
     *
     * It is implementation of function \f$\mathbf{g}_i\f$.
     */
    Eigen::Vector3d transformFeatureInverseDepthEstimateToCameraFrame(const FeatureTrack& feature_track, std::size_t i, const Eigen::Vector3d& est) const;
    
    /**
     * @brief Implementation of camera model.
     *
     * It implements pinhole camera model with radial and tangential
     * distortions;
     *
     * It is implementation of function \f$\mathbf{h}\f$
     */
    Eigen::Vector2d cameraProject(const Eigen::Vector3d& p) const;
    
    Eigen::Matrix<double, 2, 3> cameraProjectJacobian(const Eigen::Vector3d& p) const;
};

#endif //TONAV_FILTER_H
