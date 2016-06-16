//
// Created by Tomas Krejci on 5/3/16.
//

#ifndef TONAV_FILTER_H
#define TONAV_FILTER_H

#include <Eigen/Dense>

#include "calibration.h"
#include "filter_state.h"
#include "feature_tracker.h"
#include "imu_buffer.h"

class ImuItem;

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
    Filter(const Calibration& calibration);

    void initialize();

    void stepInertial(double timedelta, const ImuItem& accel, const ImuItem& gyro);
    void stepCamera(double timedelta, cv::Mat& frame);

    BodyState propagateBodyState(const BodyState& body_state_old, double timedelta,
        const ImuItem& accel, const ImuItem& gyro);
    
    void propagateRotation(
            const BodyState& old_state, BodyState& new_state, double timedelta, const ImuItem& accel,
            const ImuItem& gyro);
    void propagateVelocityAndPosition(
            const BodyState& old_state, BodyState& new_state, double timedelta, const ImuItem& accel,
            const ImuItem& gyro);

    /**
     * @brief Set initial estimate of gravity in global frame.
     *
     * This is usually calculated as averate of first few accelerometer measurements.
     * It assumes, that device don't move for few seconds at the beginning.
     *
     * @param gravity Initial estimate of gravity in global frame.
     */
    void setGlobalGravity(Eigen::Vector3d gravity);
    Eigen::Vector3d getGlobalGravity() const;

    Eigen::Vector3d getCurrentPosition();
    Eigen::Quaterniond getCurrentAttitude();
    
    /**
     * @brief Calculate \f$\hat{t} = t + \hat{t}_d\f$
     *
     * This method calculates image capture time from time when image arrived.
     *
     * @param arrive_time Time whan image arrived
     * @return Estimated image capture time
     */
    double getImageCaptureTime(double arrive_time);

private:
    Calibration calibration_;
    
    Eigen::Vector3d global_gravity_;

    FilterState filter_state_;
    Eigen::Matrix<double, 15, 15> filter_covar_;

    FeatureTracker::feature_track_list features_tracked_;

    void initializeBodyFrame();
    void initializeImuCalibration();
    void initializeCameraCalibration();
    void initializeBodyPoses();
    
    /**
     * @todo Implement this
     */
    void augment();
    void pruneCameraPoses(const FeatureTracker::feature_track_list& residualized_features);

    static Eigen::Matrix<double, 9, 1> vectorizeMatrix(const Eigen::Matrix<double, 3, 3>& mat);
    static Eigen::Matrix<double, 3, 3> unvectorizeMatrix(Eigen::Block<FilterState::StateType, 9, 1> vec);
    static Eigen::Matrix4d omegaMatrix(const Eigen::Vector3d vec);
    static Eigen::Matrix3d crossMatrix(const Eigen::Vector3d vec);

    FeatureTracker feature_tracker_;
};

#endif //TONAV_FILTER_H
