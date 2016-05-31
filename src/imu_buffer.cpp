//
// Created by Tomas Krejci on 5/11/16.
//

#include "imu_buffer.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include "exceptions/general_exception.h"
#include "exceptions/impossible_exception.h"
#include "imu_device.h"
#include "imu_item.h"

ImuBuffer::ImuBuffer(ImuDevice device, std::size_t size)
: device_(device), items_(size) {
    size_ = size;
    cur_pos_ = 0;
}

void ImuBuffer::addMeasurement(ImuItem item) {
    if (item.getDevice() != device_) {
        throw GeneralException("ImuBuffer is only for one device.");
    }
    const ImuItem& last = items_[(cur_pos_ - 1) % size_];
    if (last.getTime() >= item.getTime()) {
        throw GeneralException("Adding item with time " + std::to_string(item.getTime()) + " to buffer with latest item with time " + std::to_string(last.getTime()));
    }
    items_[cur_pos_ % size_] = item;
    cur_pos_ += 1;
}

ImuItem ImuBuffer::interpolateAtTime(double time) const {
    if (time > getMaxTime()) {
        throw GeneralException("Lookup would require extrapolation into the future.");
    }
    if (time < getMinTime()) {
        throw GeneralException("Interpolating too far in to past.");
    }
    for (int i = 0; i < size_ - 1; ++i) {
        const ImuItem& later = items_[(cur_pos_ - i - 1) % size_];
        const ImuItem& earlier = items_[(cur_pos_ - i - 2) % size_];
        if (std::abs(time - later.getTime()) < 1e-3) {
            return later;
        }
        if (std::abs(time - earlier.getTime()) < 1e-3) {
            return earlier;
        }
        if (earlier.getTime() >= later.getTime()) {
            throw ImpossibleException("Bad time ordering.");
        }
        if (earlier.getTime() < time && time < later.getTime()) {
            ImuItem interpolated;
            double time_delta = later.getTime() - earlier.getTime();
            double t = (time - earlier.getTime()) / time_delta;
            interpolated.time_ = time;
            interpolated.device_ = later.getDevice();
            interpolated.x_ = t * earlier.x_ + (1-t) * later.x_;
            interpolated.y_ = t * earlier.y_ + (1-t) * later.y_;
            interpolated.z_ = t * earlier.z_ + (1-t) * later.z_;
            return interpolated;
        }
    }
    throw ImpossibleException("Interpolation failure. Time " + std::to_string(time) + " min " + std::to_string(getMinTime()) + " max " + std::to_string(getMaxTime()));
}

double ImuBuffer::getMaxTime() const {
    if (!isReady()) {
        throw GeneralException("ImuBuffer not ready.");
    }

    const ImuItem& last = items_[(cur_pos_ - 1) % size_];
    return last.getTime();
}

double ImuBuffer::getMinTime() const {
    if (!isReady()) {
        throw GeneralException("ImuBuffer not ready.");
    }
    const ImuItem& first = items_[cur_pos_ % size_];
    return first.getTime();
}

bool ImuBuffer::isReady() const {
    return cur_pos_ >= size_;
}

