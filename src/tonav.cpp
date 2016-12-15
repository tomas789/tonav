#include "tonav.h"

#include <cmath>
#include <ros/ros.h>
#include <limits>
#include <mutex>

#include "camera_item.h"
#include "imu_device.h"
#include "state_initializer.h"
#include "stats.h"
#include "stats_timer.h"

Tonav::Tonav(std::shared_ptr<Calibration> calibration, const Eigen::Vector3d& p_B_C)
        : state_initializer_(new StateInitializer), filter_(calibration, state_initializer_) {
    filter_.setInitialBodyPositionInCameraFrame(p_B_C);
}

bool Tonav::updateAcceleration(double time, Eigen::Vector3d accel) {
    if (!accel_buffer_.empty()) {
        double last_accelerometer_time = lastAccelerometerTime();
        if (time <= last_accelerometer_time) {
            std::cout << "Got accelerometer measurement from time " << time << ", last item time is "
                << last_accelerometer_time << ". Skipping." << std::endl;
            return false;
        }
    }
    assert(!std::isnan(accel.maxCoeff()));
    ImuItem item = ImuItem::fromVector3d(time, ImuDevice::ACCELEROMETER, accel);
    accel_buffer_.push_back(item);
    return tryPropagate();
}

bool Tonav::updateRotationRate(double time, Eigen::Vector3d gyro) {
    if (!gyro_buffer_.empty()) {
        double last_gyroscope_time = lastGyroscopeTime();
        if (time <= last_gyroscope_time) {
            std::cout << "Got gyroscope measurement from time " << time << ", last item time is " << last_gyroscope_time
                << ". Skipping." << std::endl;
            return false;
        }
    }
    assert(!std::isnan(gyro.maxCoeff()));
    ImuItem item = ImuItem::fromVector3d(time, ImuDevice::GYROSCOPE, gyro);
    gyro_buffer_.push_back(item);
    return tryPropagate();
}

void Tonav::updateImage(double time, const cv::Mat& image) {
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
    
    double first_line_capture_time = filter_.getImageFirstLineCaptureTime(time);
    if (first_line_capture_time < getMinimalNextPropagationTime()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(filter_sync_);
    camera_item_ = CameraItem(time, image);
}

Quaternion Tonav::getCurrentOrientation() {
    return filter_.getCurrentAttitude();
}

Eigen::Vector3d Tonav::getCurrentPosition() {
    return filter_.getCurrentPosition();
}

Eigen::Vector3d Tonav::getCurrentVelocity() {
    return filter_.getCurrentVelocity();
}

cv::Mat Tonav::getCurrentImage() const {
    return camera_item_.cgetImage();
}

double Tonav::time() const {
    return filter_.time();
}

const Filter& Tonav::filter() const {
    return filter_;
}

const FilterState& Tonav::state() const {
    return filter_.state();
}

void Tonav::orientationCorrection(const Quaternion& orientation) {
    filter_.orientationCorrection(orientation);
}

void Tonav::positionCorrection(const Eigen::Vector3d& position) {
    filter_.positionCorrection(position);
}

void Tonav::velocityCorrection(const Eigen::Vector3d& velocity) {
    filter_.velocityCorrection(velocity);
}

std::shared_ptr<StateInitializer> Tonav::initializer() {
    return state_initializer_;
}

std::shared_ptr<const StateInitializer> Tonav::initializer() const {
    return state_initializer_;
}

bool Tonav::isInitialized() const {
    return filter_.isInitialized();
}

std::vector<Eigen::Vector3d> Tonav::featurePointCloud() const {
    return filter_.featurePointCloud();
}

Tonav::~Tonav() {
    Stats& stats = Stats::getGlobalInstance();
    std::cout << " --== STATS ==--" << std::endl;
    std::cout << stats.str() << std::endl;
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
    
    if (!filter_.isInitialized()) {
        if (accel_buffer_.empty() || gyro_buffer_.empty()) {
            return false;
        }
        
        double accel_time = accel_buffer_.front().getTime();
        double gyro_time = gyro_buffer_.front().getTime();
        
        if (accel_time < gyro_time) {
            double max_accel_time = accel_buffer_.back().getTime();
            if (max_accel_time < gyro_time) {
                return false;
            }
            accel_buffer_.truncateToMinimalInterpolationTime(gyro_time);
        } else {
            double max_gyro_time = gyro_buffer_.back().getTime();
            if (max_gyro_time <= accel_time) {
                return false;
            }
            gyro_buffer_.truncateToMinimalInterpolationTime(accel_time);
        }
        
        it_accel_ = std::begin(accel_buffer_);
        it_gyro_ = std::begin(gyro_buffer_);
        
        initialization_time_ = std::max(it_accel_->getTime(), it_gyro_->getTime());
        propagateToTime(initialization_time_);
        assert(filter_.isInitialized());
        assert(!camera_item_);
        return false;
    }
    
    if (!camera_item_ || camera_item_.wasProcessed()) {
        return false;
    }
    
    double camera_item_last_line_capture_time = filter_.getImageLastLineCaptureTime(camera_item_.getTime());
    bool accel_ok = lastAccelerometerTime() > camera_item_last_line_capture_time;
    bool gyro_ok = lastGyroscopeTime() > camera_item_last_line_capture_time;
    
    if (accel_ok && gyro_ok) {
        double camera_item_capture_time = filter_.getImageCaptureTime(camera_item_.getTime());
        bool filter_was_initialized = filter_.isInitialized();
        propagateToTime(camera_item_capture_time);
        if (filter_was_initialized) {
            Stats::getGlobalInstance().openLevel("Tonav::Update");
            update();
            camera_item_.setIsProcessed();
            Stats::getGlobalInstance().closeCurrentLevel();
        }
        return true;
    } else {
        return false;
    }
}

void Tonav::propagateToTime(double t) {
    StatsTimer timer("Tonav::propagateToTime");
    
    assert(!isInitialized() || time() < t);
    if (it_accel_->getTime() < it_gyro_->getTime()) {
        assert(!isInitialized() || it_accel_->getTime() <= time());
        assert(!isInitialized() || time() < it_gyro_->getTime());
    } else {
        assert(!isInitialized() || time() < it_accel_->getTime());
        assert(!isInitialized() || it_gyro_->getTime() <= time());
    }
    
    for (;;) {
        double step_time = getMinimalNextPropagationTime();
        
        if (time() != step_time) {
            if (it_accel_->getTime() < it_gyro_->getTime()) {
                assert(!isInitialized() || it_accel_->getTime() <= time());
                assert(!isInitialized() || time() < it_gyro_->getTime());
            } else {
                assert(!isInitialized() || time() < it_accel_->getTime());
                assert(!isInitialized() || it_gyro_->getTime() <= time());
            }
            assert(std::next(it_accel_) != std::end(accel_buffer_));
            assert(std::next(it_gyro_) != std::end(gyro_buffer_));
            ImuItem accel_item = ImuBuffer::interpolate(step_time, *it_accel_, *std::next(it_accel_));
            ImuItem gyro_item = ImuBuffer::interpolate(step_time, *it_gyro_, *std::next(it_gyro_));
            filter_.stepInertial(step_time, accel_item, gyro_item);
        }
        
        if (std::min(std::next(it_accel_)->getTime(), std::next(it_gyro_)->getTime()) > t) {
            ImuItem accel_item = ImuBuffer::interpolate(t, *it_accel_, *std::next(it_accel_));
            ImuItem gyro_item = ImuBuffer::interpolate(t, *it_gyro_, *std::next(it_gyro_));
            filter_.stepInertial(t, accel_item, gyro_item);
            incrementBufferPointer();
            if ((std::next(it_accel_)->getTime() == t) || (std::next(it_gyro_)->getTime() == t)) {
                incrementBufferPointer();
            }
            break;
        }
        
        incrementBufferPointer();
    }
    
    if (it_accel_->getTime() < it_gyro_->getTime()) {
        assert(!isInitialized() || it_accel_->getTime() <= time());
        assert(!isInitialized() || time() < it_gyro_->getTime());
    }
}

void Tonav::update() {
    assert(filter_.isInitialized());
    filter_.stepCamera(camera_item_.getTime(), camera_item_.getImage(), it_gyro_, it_accel_);
}

void Tonav::incrementBufferPointer() {
    if (std::next(it_accel_)->getTime() < std::next(it_gyro_)->getTime()) {
        ++it_accel_;
        assert(it_accel_ != std::end(accel_buffer_));
    } else {
        ++it_gyro_;
        assert(it_gyro_ != std::end(gyro_buffer_));
    }
}

double Tonav::lastGyroscopeTime() const {
    const ImuItem& last_gyro = gyro_buffer_.back();
    return last_gyro.getTime();
}

double Tonav::lastAccelerometerTime() const {
    const ImuItem& last_accel = accel_buffer_.back();
    return last_accel.getTime();
}

double Tonav::getMinimalNextPropagationTime() const {
    return std::max(it_accel_->getTime(), it_gyro_->getTime());
}
