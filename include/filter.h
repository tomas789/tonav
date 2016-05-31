//
// Created by Tomas Krejci on 5/3/16.
//

#ifndef TONAV_FILTER_H
#define TONAV_FILTER_H

#include <Eigen/Dense>

#include "calibration.h"
#include "filter_state.h"

class ImuItem;

namespace cv {
    class Mat;
}

class Filter {
    constexpr static std::size_t poses_ = FilterState::getPoses();
public:
    Filter(const Calibration& calibration);

    void initialize();

    void stepInertial(double timedelta, const ImuItem& accel, const ImuItem& gyro);
    void stepCamera(double timedelta, const cv::Mat& frame);

    void propagateRotation(
            FilterState& old_state, FilterState& new_state, double timedelta, const ImuItem& accel,
            const ImuItem& gyro);
    void propagateVelocityAndPosition(
            FilterState& old_state, FilterState& new_state, double timedelta, const ImuItem& accel,
            const ImuItem& gyro);

    Eigen::Vector3d getGlobalGravity() const;

    Eigen::Vector3d getCurrentPosition();
    Eigen::Quaterniond getCurrentAttitude();

    constexpr static std::size_t getPoses() {
        return poses_;
    }

private:
    Calibration calibration_;

    FilterState filter_state_;
    Eigen::Matrix<double, 15, 15> filter_covar_;

    void initializeBodyFrame();
    void initializeImuCalibration();
    void initializeCameraCalibration();
    void initializeBodyPoses();

    static Eigen::Matrix<double, 9, 1> vectorizeMatrix(const Eigen::Matrix<double, 3, 3>& mat);
    static Eigen::Matrix<double, 3, 3> unvectorizeMatrix(Eigen::Block<FilterState::StateType, 9, 1> vec);
    static Eigen::Matrix4d omegaMatrix(const Eigen::Vector3d vec);
    static Eigen::Matrix3d crossMatrix(const Eigen::Vector3d vec);
};

#endif //TONAV_FILTER_H
