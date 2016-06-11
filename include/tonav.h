#ifndef TONAV_TONAV_H
#define TONAV_TONAV_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include "calibration.h"
#include "filter.h"
#include "imu_buffer.h"
#include "imu_device.h"

/**
 * @brief This is main class for communicating with filter. 
 *        You can pass IMU and camera data to filter through
 *        it and get information about current filter state.
 *
 * As a user, you should only communicate with this class.
 */
class Tonav {
public:
    /**
     * @brief Initialize filter with configuration
     *
     * @param calibration Calibration
     */
    Tonav(Calibration& calibration);
    
    /**
     * @brief Perform navigation step with data from accelerometer.
     *
     * Parameter \p accel is vector \f$(x, y, z)\f$ measured
     * in \f$\frac{m}{s^2}\f$.
     *
     * @param time Time at which accelerometer data were captured.
     * @param accel 3D vector of accelerometer data.
     */
    void updateAcceleration(double time, Eigen::Vector3d accel);
    
    /**
     * @brief Perform navigation step with data from gyroscope.
     *
     * Parameter \p gyro is vector \f$(x, y, z)\f$ measured
     * in \f$\frac{rad}{s}\f$.
     *
     * @param time Time at which gyroscope data were captured.
     * @param gyro 3D vector of gyroscope data.
     */
    void updateRotationRate(double time, Eigen::Vector3d gyro);
    
    /**
     * @brief Perform navigation step with data from both accelerometer
     * and gyroscore.
     *
     * Parameter \p accel is vector \f$(x, y, z)\f$ measured
     * in \f$\frac{m}{s^2}\f$. Parameter \p gyro is vector 
     * \f$(x, y, z)\f$ measured in \f$\frac{rad}{s}\f$.
     *
     * @param time Time at which accelerometer and gyroscope data 
     *        were  both captured.
     * @param accel 3D vector of accelerometer data.
     * @param gyro 3D vector of gyroscope data.
     */
    void updateAccelerationAndRotationRate(double time, Eigen::Vector3d accel, Eigen::Vector3d gyro);
    
    /**
     * @brief Perform navigation step with image from camera.
     *
     * Time passed to this function should be as deterministic as
     * possible. Try to avoid as much buffering as you can. Also
     * don't do any variable-duration image processing like detecting
     * features from image.
     *
     * Image should by in format GRAY8 and should have reasonable
     * resolution. Full-HD is unnecessarily large. Something about
     * 640x480 is good enough. It speeds up processing of the image
     * and delivers more real-time localization.
     *
     * @param time Time at which camera image was captured.
     * @param image Captured image.
     */
    void updateImage(double time, cv::Mat& image);
    
    /**
     * @brief Set camera model params.
     *
     * Tonav is using pinhole camera modes as implemented in OpenCV.
     * Using this function, you can set camera intrinsics parameters.
     *
     * Camera matrix:
     *
     *  \f$
     *    \begin{pmatrix}
     *      f_x & 0 & c_x \\
     *      0 & f_y & c_y \\
     *      0 & 0 & 1
     *    \end{pmatrix}
     *  \f$
     *
     * Distortion params:
     *
     *  \f$
     *    \begin{pmatrix}
     *      k_1 & k_2 & p_1 & p_2 & k_3
     *    \end{pmatrix}
     *  \f$
     *
     *
     * @param camera_matrix Camera matrix
     * @param distortion_params Distortion parameters
     */
    void setCameraModelParams(const Eigen::Matrix3d& camera_matrix, const Eigen::Matrix<double, 5, 1> distortion_params);
    
    bool filterWasUpdated() const;
    
    Eigen::Quaterniond getCurrentOrientation();
    Eigen::Vector3d getCurrentPosition();
    cv::Mat getCurrentImage() const;
    
private:
    std::size_t initial_delay_;
    Filter filter_;
    
    bool has_global_gravity_set_;
    
    double last_image_capture_time_;
    cv::Mat last_image_;
    
    double last_update_time_;
    bool filter_was_updated_;
    
    std::list<ImuItem> accel_buffer_;
    std::list<ImuItem> gyro_buffer_;
    
    bool checkGlobalGravity();
    double getMaxAccelerometerTime() const;
    double getMaxGyroscopeTime() const;
    
    void initialize();
    void initializeGlobalGravity();
    void initializeLastUpdateTime();
    
    void updateAccelerationImpl(double time, Eigen::Vector3d accel);
    void updateRotationRateImpl(double time, Eigen::Vector3d gyro);
    
    void performUpdateIfPossible();
    void performUpdate();
    
    ImuItem interpolate(double time, const ImuItem& earlier, const ImuItem& later) const;
};

#endif //TONAV_TONAV_H
