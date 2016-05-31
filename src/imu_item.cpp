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
                if (!Calibration::tryParseDouble(s, item.x_)) {
                    throw GeneralException("Failed parse IMU data: Expected double value: " + s);
                }
                break;
            case 3:
                // y
                if (!Calibration::tryParseDouble(s, item.y_)) {
                    throw GeneralException("Failed parse IMU data: Expected double value: " + s);
                }
                break;
            case 4:
                // z
                if (!Calibration::tryParseDouble(s, item.z_)) {
                    throw GeneralException("Failed parse IMU data: Expected double value: " + s);
                }
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

ImuDevice ImuItem::getDevice() const {
    return device_;
}

double ImuItem::getTime() const {
    return time_;
}

double ImuItem::getX() const {
    return x_;
}

double ImuItem::getY() const {
    return y_;
}

double ImuItem::getZ() const {
    return z_;
}