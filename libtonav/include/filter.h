//
// Created by Tomas Krejci on 5/3/16.
//

#ifndef TONAV_FILTER_H
#define TONAV_FILTER_H

#include <Eigen/Core>
#include <functional>

#include "body_state.h"
#include "calibration.h"
#include "camera_algorithms.h"
#include "feature_rezidualization_result.h"
#include "feature_tracker.h"
#include "filter_state.h"
#include "imu_buffer.h"
#include "state_initializer.h"
#include "quaternion.h"

class ImuItem;
class CameraReprojectionFunctor;

namespace cv {
    class Mat;
}

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
    friend std::ostream& operator<< (std::ostream& out, Filter& filter);
    
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
    Quaternion getCurrentAttitude();
    
    /** @brief Get current estimated velocity */
    Eigen::Vector3d getCurrentVelocity();
    
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
    
    Quaternion getBodyToCameraRotation() const;
    
    Eigen::Vector3d getPositionOfBodyInCameraFrame() const;
    
    void orientationCorrection(const Quaternion& orientation);
    void positionCorrection(const Eigen::Vector3d& position);
    void velocityCorrection(const Eigen::Vector3d& velocity);
    
    /**
     * @brief Get a reference to currently valid filter state.
     */
    const FilterState& state() const;

    const Calibration& calibration() const;
    
    void setInitialBodyPositionInCameraFrame(const Eigen::Vector3d& position);
    
    std::vector<Eigen::Vector3d> featurePointCloud() const;
    
    const CameraAlgorithms& cameraAlgorithms() const;
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
    
    CameraAlgorithms camera_algorithms_;
    
    /** @brief Number of rows in last camera image */
    std::size_t frame_rows_;
    
    std::vector<Eigen::Vector3d> feature_positions_;
    
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
    
    FeatureRezidualizationResult rezidualizeFeature(const FeatureTrack& feature_track, cv::Mat& frame) const;
    
    void performUpdate(const FeatureTracker::feature_track_list& features_to_rezidualize, cv::Mat& frame);
    
    bool gatingTest(const Eigen::VectorXd& r_0_i, const Eigen::MatrixXd H_0_i);
    
    void updateState(const Eigen::MatrixXd& T_H, const Eigen::VectorXd& r_q, const Eigen::MatrixXd& H, const Eigen::VectorXd& r);
};

std::ostream& operator<< (std::ostream& out, Filter& filter);

#endif //TONAV_FILTER_H
