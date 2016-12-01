#include "tonav.h"

#include <cmath>
#include <ros/ros.h>
#include <limits>
#include <mutex>

#include "camera_item.h"
#include "imu_device.h"

Tonav::Tonav(std::shared_ptr<Calibration> calibration, const Eigen::Quaterniond& orientation, const Eigen::Vector3d& position, const Eigen::Vector3d& p_B_C) : filter_(calibration) {
    filter_.setInitialOrientation(orientation);
    filter_.setInitialPosition(position);
    filter_.setInitialBodyPositionInCameraFrame(p_B_C);
    filter_.setGyroscopeInterpolationCallback(std::bind(&Tonav::interpolate_any_time, this, std::placeholders::_1, ImuDevice::GYROSCOPE));
    filter_.setAccelerometerInterpolationCallback(std::bind(&Tonav::interpolate_any_time, this, std::placeholders::_1, ImuDevice::ACCELEROMETER));
    next_image_allowed_time_ = 0;
}

bool Tonav::updateAcceleration(double time, Eigen::Vector3d accel) {
    if (time <= filter_.time()) {
        std::cout << "Got accelerometer measurement from time " << time << ", filter time is " << filter_.time() << "Skipping." << std::endl;
        return false;
    }
    ImuItem item = ImuItem::fromVector3d(time, ImuDevice::ACCELEROMETER, accel);
    accel_buffer_[time] = item;
    return tryPropagate();
}

bool Tonav::updateRotationRate(double time, Eigen::Vector3d gyro) {
    if (time <= filter_.time()) {
        std::cout << "Got gyroscope measurement from time " << time << ", filter time is " << filter_.time() << "Skipping." << std::endl;
        return false;
    }
    ImuItem item = ImuItem::fromVector3d(time, ImuDevice::GYROSCOPE, gyro);
    gyro_buffer_[time] = item;
    return tryPropagate();
}

void Tonav::updateImage(double time, cv::Mat image) {
    if (!filter_.isInitialized()) {
        return;
    }
    
    double first_image_line_capture_time = filter_.getImageFirstLineCaptureTime(time);
    if (first_image_line_capture_time < initialization_time_) {
        return;
    }
    
    if (camera_item_ && !camera_item_.wasProcessed()) {
        return;
    }
    
    if (time < next_image_allowed_time_) {
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(filter_sync_);
        camera_item_ = CameraItem(time, image);
    }
}

Eigen::Quaterniond Tonav::getCurrentOrientation() {
    return filter_.getCurrentAttitude();
}

Eigen::Vector3d Tonav::getCurrentPosition() {
    return filter_.getCurrentPosition();
}

cv::Mat Tonav::getCurrentImage() const {
    return camera_item_.getImage();
}

double Tonav::time() const {
    return filter_.time();
}

//void Tonav::performUpdateIfPossible() {
//    if (!filter_.isInitialized()) {
//        return;
//    }
//    bool has_image = !std::isnan(last_image_capture_time_);
//    bool has_accelerometer_after_image = getMaxAccelerometerTime() >= last_image_capture_time_;
//    bool has_gyroscope_after_image = getMaxGyroscopeTime() >= last_image_capture_time_;
//    
//    if (has_image && has_accelerometer_after_image && has_gyroscope_after_image) {
//        performUpdate();
//        filter_was_updated_ = true;
//    } else {
//        filter_was_updated_ = false;
//    }
//}

//void Tonav::performUpdate() {
//    ROS_INFO("PERFORMING UPDATE");
//    
//    double accelerometer_time = accel_buffer_.front().getTime();
//    double gyroscope_time = gyro_buffer_.front().getTime();
//    
//    ImuItem accelerometer_at_image_time;
//    ImuItem gyroscope_at_image_time;
//    
//    while (accelerometer_time < last_image_capture_time_ || gyroscope_time < last_image_capture_time_) {
//        // Propagate IMU
//        if (accelerometer_time < gyroscope_time) {
//            std::list<ImuItem>::iterator earlier_it = std::begin(accel_buffer_);
//            std::list<ImuItem>::iterator later_it = ++std::begin(accel_buffer_);
//            
//            if (earlier_it->getTime() <= last_image_capture_time_ && later_it->getTime() > last_image_capture_time_) {
//                accelerometer_at_image_time = interpolate(last_image_capture_time_, *earlier_it, *later_it);
//            }
//            
//            ImuItem accelerometer = interpolate(gyroscope_time, *earlier_it, *later_it);
//            filter_.stepInertial(gyroscope_time - last_update_time_, accelerometer, gyro_buffer_.front());
//            last_update_time_ = gyroscope_time;
//            accel_buffer_.pop_front();
//            accelerometer_time = accel_buffer_.front().getTime();
//        } else {
//            std::list<ImuItem>::iterator earlier_it = std::begin(gyro_buffer_);
//            std::list<ImuItem>::iterator later_it = ++std::begin(gyro_buffer_);
//            
//            if (earlier_it->getTime() <= last_image_capture_time_ && later_it->getTime() > last_image_capture_time_) {
//                gyroscope_at_image_time = interpolate(last_image_capture_time_, *earlier_it, *later_it);
//            }
//            
//            ImuItem gyroscope = interpolate(accelerometer_time, *earlier_it, *later_it);
//            filter_.stepInertial(accelerometer_time - last_update_time_, accel_buffer_.front(), gyroscope);
//            last_update_time_ = accelerometer_time;
//            gyro_buffer_.pop_front();
//            gyroscope_time = gyro_buffer_.front().getTime();
//        }
//    }
//    
//    // Update image
//    filter_.stepInertial(last_image_capture_time_ - last_update_time_, accelerometer_at_image_time, gyroscope_at_image_time);
//    filter_.stepCamera(last_image_capture_time_ - last_update_time_, last_image_);
//    last_update_time_ = last_image_capture_time_;
//    last_image_capture_time_ = NAN;
//}

bool Tonav::tryPropagate() {
    std::lock_guard<std::mutex> lock(filter_sync_);
    
    if (accel_buffer_.size() < 10 || gyro_buffer_.size() < 10) {
        return false;
    }
    
    if (!filter_.isInitialized()) {
        double min_accel_time = accel_buffer_.begin()->first;
        double min_gyro_time = gyro_buffer_.begin()->first;
        initialization_time_ = std::max(min_accel_time, min_gyro_time);
        std::map<double, ImuItem>& longer_buffer = min_accel_time < min_gyro_time ? accel_buffer_ : gyro_buffer_;
        auto lower_it = longer_buffer.begin();
        // `upper_it` points to first element that should be kept
        auto upper_it = longer_buffer.lower_bound(initialization_time_);
        longer_buffer.erase(lower_it, upper_it);
        
        propagateToTime(initialization_time_);
        assert(filter_.isInitialized());
        assert(!camera_item_);
        return false;
    }
    
    if (!camera_item_ || camera_item_.wasProcessed()) {
        return false;
    }
    
    double camera_item_last_line_capture_time = filter_.getImageLastLineCaptureTime(camera_item_.getTime());
    bool accel_ok = accel_buffer_.rbegin()->first > camera_item_last_line_capture_time;
    bool gyro_ok = gyro_buffer_.rbegin()->first > camera_item_last_line_capture_time;
    
    if (accel_ok && gyro_ok) {
        double camera_item_capture_time = filter_.getImageCaptureTime(camera_item_.getTime());
        bool filter_was_initialized = filter_.isInitialized();
        propagateToTime(camera_item_capture_time);
        if (filter_was_initialized) {
            update();
            camera_item_.setIsProcessed();
        }
        return true;
    } else {
        return false;
    }
}

void Tonav::propagateToTime(double time) {
    std::map<double, ImuItem>::iterator it_accel = accel_buffer_.lower_bound(filter_.time());
    std::map<double, ImuItem>::iterator it_gyro = gyro_buffer_.lower_bound(filter_.time());
    
    double step_time = std::min(it_accel->first, it_gyro->first);
    
    std::size_t iter = 0;
    while (true) {
        assert(it_accel != std::end(accel_buffer_));
        assert(std::next(it_accel) != std::end(accel_buffer_));
        assert(it_gyro != std::end(gyro_buffer_));
        assert(std::next(it_gyro) != std::end(gyro_buffer_));
        
        ImuItem accel_item = interpolate(step_time, it_accel->second, std::next(it_accel)->second);
        ImuItem gyro_item = interpolate(step_time, it_gyro->second, std::next(it_gyro)->second);
        
        filter_.stepInertial(step_time, accel_item, gyro_item);
        
        if (step_time == time) {
            break;
        } else {
            step_time = std::min(std::next(it_accel)->first, std::next(it_gyro)->first);
            if (step_time > time) {
                step_time = time;
            } else if (std::abs(it_accel->first - it_gyro->first) < 1e-12 && std::abs(std::next(it_accel)->first - std::next(it_gyro)->first) < 1e-12) {
                ++it_accel;
                ++it_gyro;
            } else if (std::next(it_accel)->first <= std::next(it_gyro)->first) {
                ++it_accel;
            } else {
                ++it_gyro;
            }
            assert(it_accel->first <= step_time && step_time <= std::next(it_accel)->first);
            assert(it_gyro->first <= step_time && step_time <= std::next(it_gyro)->first);
        }
        iter += 1;
    }
    
    if (std::abs(filter_.time() - time) >= 1e-6) {
        std::cerr << "Filter time: " << filter_.time() << std::endl;
        std::cerr << "Time: " << time << std::endl;
        assert(std::abs(filter_.time() - time) < 1e-6);
    }
    
    next_image_allowed_time_ = std::max(std::next(it_accel)->first, std::next(it_gyro)->first);
}

void Tonav::update() {
    assert(filter_.isInitialized());
    filter_.stepCamera(camera_item_.getTime(), camera_item_.getImage());
}

ImuItem Tonav::interpolate(double time, const ImuItem &earlier, const ImuItem &later) const {
    double time_delta = later.getTime() - earlier.getTime();
    double t = (time - earlier.getTime()) / time_delta;
    ImuDevice device = later.getDevice();
    Eigen::Vector3d data = t * earlier.getVector() + (1-t) * later.getVector();
    ImuItem interpolated = ImuItem::fromVector3d(time, device, data);
    return interpolated;
}

ImuItem Tonav::interpolate_any_time(double time, ImuDevice device) {
    const std::map<double, ImuItem>& buffer = device == ImuDevice::ACCELEROMETER ? accel_buffer_ : gyro_buffer_;
    auto later = buffer.lower_bound(time);
    auto earlier = std::prev(later);
    return interpolate(time, earlier->second, later->second);
}
