#include "tonav.h"

#include <cmath>
#include <ros/ros.h>
#include <limits>

Tonav::Tonav(Calibration& calibration) : filter_(calibration) {
    has_global_gravity_set_ = false;
    last_update_time_ = NAN;
    filter_was_updated_ = false;
}

void Tonav::updateAcceleration(double time, Eigen::Vector3d accel) {
    ROS_DEBUG("Received updateAcceleration");
    updateAccelerationImpl(time, accel);
}

void Tonav::updateRotationRate(double time, Eigen::Vector3d gyro) {
    ROS_DEBUG("Received updateRotationRate");
    updateRotationRateImpl(time, gyro);
}

void Tonav::updateAccelerationAndRotationRate(double time, Eigen::Vector3d accel, Eigen::Vector3d gyro) {
    ROS_DEBUG("Received updateAccelerationAndRotationRate");
    updateAccelerationImpl(time, accel);
    updateRotationRateImpl(time, gyro);
}

void Tonav::updateImage(double time, cv::Mat &image) {
    ROS_DEBUG("Received updateImage");
    
    if (!std::isnan(last_image_capture_time_) || !checkGlobalGravity()) {
        return;
    }
    
    last_image_capture_time_ = filter_.getImageCaptureTime(time);
    last_image_ = image;
    
    performUpdateIfPossible();
}

bool Tonav::filterWasUpdated() const {
    return filter_was_updated_;
}

Eigen::Quaterniond Tonav::getCurrentOrientation() {
    return filter_.getCurrentAttitude();
}

Eigen::Vector3d Tonav::getCurrentPosition() {
    return filter_.getCurrentPosition();
}

cv::Mat Tonav::getCurrentImage() const {
    return last_image_;
}

bool Tonav::checkGlobalGravity() {
    if (has_global_gravity_set_) {
        return true;
    }
    
    if (accel_buffer_.size() < 30) {
        return false;
    }
    
    initialize();
    return true;
}

double Tonav::getMaxAccelerometerTime() const {
    if (accel_buffer_.empty()) {
        return -1 * std::numeric_limits<double>::infinity();
    }
    return accel_buffer_.back().getTime();
}

double Tonav::getMaxGyroscopeTime() const {
    if (gyro_buffer_.empty()) {
        return -1 * std::numeric_limits<double>::infinity();
    }
    return gyro_buffer_.back().getTime();
}

void Tonav::initialize() {
    initializeGlobalGravity();
    initializeLastUpdateTime();
    
    filter_.initialize();
}

void Tonav::initializeGlobalGravity() {
    Eigen::Vector3d global_gravity;
    global_gravity.setZero();
    for (auto it = std::begin(accel_buffer_); it != std::end(accel_buffer_); ++it) {
        global_gravity += it->getVector();
    }
    global_gravity /= accel_buffer_.size();
    
    filter_.setGlobalGravity(global_gravity);
    has_global_gravity_set_ = true;
    
    ROS_DEBUG_STREAM("Global gravity: " << global_gravity);
    ROS_ASSERT(accel_buffer_.size() > 1);
    ROS_ASSERT(gyro_buffer_.size() > 1);
    
    std::list<ImuItem>::iterator keep_last_accel_iterator = std::begin(accel_buffer_);
    std::advance(keep_last_accel_iterator, accel_buffer_.size() - 1);
    accel_buffer_.erase(std::begin(accel_buffer_), keep_last_accel_iterator);
    
    std::list<ImuItem>::iterator keep_last_gyro_iterator = std::begin(gyro_buffer_);
    std::advance(keep_last_gyro_iterator, gyro_buffer_.size() - 1);
    gyro_buffer_.erase(std::begin(gyro_buffer_), keep_last_gyro_iterator);
}

void Tonav::initializeLastUpdateTime() {
    last_update_time_ = std::min(accel_buffer_.front().getTime(), gyro_buffer_.front().getTime());
}

void Tonav::updateAccelerationImpl(double time, Eigen::Vector3d accel) {
    ImuItem item = ImuItem::fromVector3d(time, ImuDevice::ACCELEROMETER, accel);
    accel_buffer_.push_back(item);
    
    if (!checkGlobalGravity()) {
        return;
    }
    
    performUpdateIfPossible();
}

void Tonav::updateRotationRateImpl(double time, Eigen::Vector3d gyro) {
    ImuItem item = ImuItem::fromVector3d(time, ImuDevice::GYROSCOPE, gyro);
    gyro_buffer_.push_back(item);
    
    if (!checkGlobalGravity()) {
        return;
    }
    
    performUpdateIfPossible();
}

void Tonav::performUpdateIfPossible() {
    bool has_image = !std::isnan(last_image_capture_time_);
    bool has_accelerometer_after_image = getMaxAccelerometerTime() >= last_image_capture_time_;
    bool has_gyroscope_after_image = getMaxGyroscopeTime() >= last_image_capture_time_;
    
    if (has_image && has_accelerometer_after_image && has_gyroscope_after_image) {
        performUpdate();
        filter_was_updated_ = true;
    } else {
        filter_was_updated_ = false;
    }
}

void Tonav::performUpdate() {
    ROS_INFO("PERFORMING UPDATE");
    
    double accelerometer_time = accel_buffer_.front().getTime();
    double gyroscope_time = gyro_buffer_.front().getTime();
    
    ImuItem accelerometer_at_image_time;
    ImuItem gyroscope_at_image_time;
    
    while (accelerometer_time < last_image_capture_time_ || gyroscope_time < last_image_capture_time_) {
        // Propagate IMU
        if (accelerometer_time < gyroscope_time) {
            std::list<ImuItem>::iterator earlier_it = std::begin(accel_buffer_);
            std::list<ImuItem>::iterator later_it = ++std::begin(accel_buffer_);
            
            if (earlier_it->getTime() <= last_image_capture_time_ && later_it->getTime() > last_image_capture_time_) {
                accelerometer_at_image_time = interpolate(last_image_capture_time_, *earlier_it, *later_it);
            }
            
            ImuItem accelerometer = interpolate(gyroscope_time, *earlier_it, *later_it);
            filter_.stepInertial(gyroscope_time - last_update_time_, accelerometer, gyro_buffer_.front());
            last_update_time_ = gyroscope_time;
            accel_buffer_.pop_front();
            accelerometer_time = accel_buffer_.front().getTime();
        } else {
            std::list<ImuItem>::iterator earlier_it = std::begin(gyro_buffer_);
            std::list<ImuItem>::iterator later_it = ++std::begin(gyro_buffer_);
            
            if (earlier_it->getTime() <= last_image_capture_time_ && later_it->getTime() > last_image_capture_time_) {
                gyroscope_at_image_time = interpolate(last_image_capture_time_, *earlier_it, *later_it);
            }
            
            ImuItem gyroscope = interpolate(accelerometer_time, *earlier_it, *later_it);
            filter_.stepInertial(accelerometer_time - last_update_time_, accel_buffer_.front(), gyroscope);
            last_update_time_ = accelerometer_time;
            gyro_buffer_.pop_front();
            gyroscope_time = gyro_buffer_.front().getTime();
        }
    }
    
    // Update image
    filter_.stepCamera(last_image_capture_time_ - last_update_time_, accelerometer_at_image_time, gyroscope_at_image_time, last_image_);
    last_update_time_ = last_image_capture_time_;
    last_image_capture_time_ = NAN;
}

ImuItem Tonav::interpolate(double time, const ImuItem &earlier, const ImuItem &later) const {
    double time_delta = later.getTime() - earlier.getTime();
    double t = (time - earlier.getTime()) / time_delta;
    ImuDevice device = later.getDevice();
    Eigen::Vector3d data = t * earlier.getVector() + (1-t) * later.getVector();
    ImuItem interpolated = ImuItem::fromVector3d(time, device, data);
    return interpolated;
}