//
// Created by Tomas Krejci on 5/11/16.
//

#include "imu_buffer.h"

#include "imu_item.h"
#include "imu_device.h"

namespace tonav {

ImuBuffer::iterator ImuBuffer::begin() {
    return std::begin(buffer_);
}

ImuBuffer::iterator ImuBuffer::end() {
    return std::end(buffer_);
}

ImuBuffer::const_iterator ImuBuffer::begin() const {
    return std::begin(buffer_);
}

ImuBuffer::const_iterator ImuBuffer::end() const {
    return std::end(buffer_);
}

ImuItem &ImuBuffer::front() {
    return buffer_.front();
}

const ImuItem &ImuBuffer::front() const {
    return buffer_.front();
}

ImuItem &ImuBuffer::back() {
    return buffer_.back();
}

const ImuItem &ImuBuffer::back() const {
    return buffer_.back();
}

bool ImuBuffer::empty() const {
    return buffer_.empty();
}

std::size_t ImuBuffer::size() const {
    return buffer_.size();
}

void ImuBuffer::push_back(const ImuItem &item) {
    buffer_.push_back(item);
}

void ImuBuffer::push_back(ImuItem &&item) {
    buffer_.push_back(std::move(item));
}

void ImuBuffer::truncateToMinimalInterpolationTime(double time) {
    while (std::next(std::begin(buffer_))->getTime() < time) {
        buffer_.pop_front();
    }
}

ImuItem ImuBuffer::interpolate(double time, const ImuItem &earlier, const ImuItem &later) {
    double time_delta = later.getTime() - earlier.getTime();
    double t = (time - earlier.getTime()) / time_delta;
    ImuDevice device = later.getDevice();
    Eigen::Vector3d data = t * earlier.getVector() + (1 - t) * later.getVector();
    ImuItem interpolated = ImuItem::fromVector3d(time, device, data);
    
    // assert(interpolated.getVector().norm() > 1e-100);
    assert(interpolated.getVector().norm() < 1e100);
    
    return interpolated;
}

ImuItem ImuBuffer::interpolateAnyTime(double time, ImuBuffer::iterator hint) {
    // Hint might be ok.
    double hint_time = hint->getTime();
    if (time == hint_time) {
        return *hint;
    }
    
    ImuBuffer::iterator it = hint;
    shiftToInterpolationInterval(time, it);
    return interpolate(time, *it, *std::next(it));
}

void ImuBuffer::shiftToInterpolationInterval(double time, ImuBuffer::iterator &it) {
    int diff = it->getTime() < time ? 1 : -1;
    while (it->getTime() > time || time >= std::next(it)->getTime()) {
        std::advance(it, diff);
    }
}

}
