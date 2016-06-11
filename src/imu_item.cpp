//
// Created by Tomas Krejci on 5/10/16.
//

#include "calibration.h"
#include "exceptions/general_exception.h"
#include "imu_item.h"

ImuItem ImuItem::fromString(std::string line) {
    ImuItem item;
    std::size_t pos = 0;
    for (int i = 0; i < 5; ++i) {
        if (pos == std::string::npos) {
            throw GeneralException("Expected 5 columns in IMU data file.");
        }
        std::size_t pos_end = line.find(';', pos);
        std::string s = line.substr(pos, pos_end - pos);
        switch (i) {
            case 0:
                // Time
                if (!Calibration::tryParseDouble(s, item.time_)) {
                    throw GeneralException("Failed parse IMU data: Expected double value: " + s);
                }
                break;
            case 1:
                // Device
                if (s == "\"accelerometer\"") {
                    item.device_ = ImuDevice::ACCELEROMETER;
                } else if (s == "\"gyroscope\"") {
                    item.device_ = ImuDevice::GYROSCOPE;
                } else {
                    throw GeneralException("Failed parse IMU data: Unknown device: " + s);
                }
                break;
            case 2:
                // x
                double x;
                if (!Calibration::tryParseDouble(s, x)) {
                    throw GeneralException("Failed parse IMU data: Expected double value: " + s);
                }
                item.data_(0, 0) = x;
                break;
            case 3:
                // y
                double y;
                if (!Calibration::tryParseDouble(s, y)) {
                    throw GeneralException("Failed parse IMU data: Expected double value: " + s);
                }
                item.data_(1, 0) = y;
                break;
            case 4:
                // z
                double z;
                if (!Calibration::tryParseDouble(s, z)) {
                    throw GeneralException("Failed parse IMU data: Expected double value: " + s);
                }
                item.data_(2, 0) = z;
                break;
            default:
                throw GeneralException("");
        }
        pos = pos_end == std::string::npos
              ? std::string::npos
              : pos_end + 1;
    }
    return item;
}

ImuItem ImuItem::fromVector3d(double time, const ImuDevice& device, const Eigen::Vector3d &data) {
    ImuItem item;
    item.time_ = time;
    item.device_ = device;
    item.data_ = data;
    return item;
}

ImuDevice ImuItem::getDevice() const {
    return device_;
}

double ImuItem::getTime() const {
    return time_;
}

double ImuItem::getX() const {
    return data_(0, 0);
}

double ImuItem::getY() const {
    return data_(1, 0);
}

double ImuItem::getZ() const {
    return data_(2, 0);
}

Eigen::Vector3d ImuItem::getVector() const {
    return data_;
}