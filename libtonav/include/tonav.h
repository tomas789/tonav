#ifndef TONAV_TONAV_H
#define TONAV_TONAV_H

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <fstream>
#include <list>
#include <mutex>

#include "camera_item.h"
#include "filter.h"
#include "imu_buffer.h"
#include "quaternion.h"

class Calibration;
class FilterState;
class StateInitializer;

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
    Tonav(std::shared_ptr<Calibration> calibration, const Eigen::Vector3d& p_B_C);
    
    /**
     * @brief Perform navigation step with data from accelerometer.
     *
     * Parameter \p accel is vector \f$(x, y, z)\f$ measured
     * in \f$\frac{m}{s^2}\f$.
     *
     * @param time Time at which accelerometer data were captured.
     * @param accel 3D vector of accelerometer data.
     * @return `true` if filter was update, `false` otherwise.
     */
    bool updateAcceleration(double time, Eigen::Vector3d accel);
    
    /**
     * @brief Perform navigation step with data from gyroscope.
     *
     * Parameter \p gyro is vector \f$(x, y, z)\f$ measured
     * in \f$\frac{rad}{s}\f$.
     *
     * @param time Time at which gyroscope data were captured.
     * @param gyro 3D vector of gyroscope data.
     * @return `true` if filter was update, `false` otherwise.
     */
    bool updateRotationRate(double time, Eigen::Vector3d gyro);
    
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
    void updateImage(double time, const cv::Mat& image);
    
    Quaternion getCurrentOrientation();
    Eigen::Vector3d getCurrentPosition();
    Eigen::Vector3d getCurrentVelocity();
    cv::Mat getCurrentImage() const;
    double time() const;
    
    const Filter& filter() const;
    const FilterState& state() const;
    
    void orientationCorrection(const Quaternion& orientation);
    void positionCorrection(const Eigen::Vector3d& position);
    void velocityCorrection(const Eigen::Vector3d& velocity);
    
    std::shared_ptr<StateInitializer> initializer();
    std::shared_ptr<const StateInitializer> initializer() const;
    
    bool isInitialized() const;
    
    std::vector<Eigen::Vector3d> featurePointCloud() const;
    
    ~Tonav();
private:
    std::shared_ptr<StateInitializer> state_initializer_;
    
    Filter filter_;
    std::mutex filter_sync_;
        
    CameraItem camera_item_;

    double initialization_time_;
    
    ImuBuffer accel_buffer_;
    ImuBuffer gyro_buffer_;
    ImuBuffer::iterator it_accel_;
    ImuBuffer::iterator it_gyro_;
    
    bool tryPropagate();
    void propagateToTime(double t);
    void update();
    
    void incrementBufferPointer();
    double getMinimalNextPropagationTime() const;
    
    double lastGyroscopeTime() const;
    double lastAccelerometerTime() const;
};

#endif //TONAV_TONAV_H
