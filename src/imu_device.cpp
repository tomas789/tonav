//
// Created by Tomas Krejci on 5/10/16.
//

#include "imu_device.h"

#include <iostream>

#include "exceptions/impossible_exception.h"

std::ostream& operator<< (std::ostream& out, const ImuDevice& device) {
    switch (device) {
        case ImuDevice::ACCELEROMETER:
            out << "IMU::Accelerometer";
            break;
        case ImuDevice::GYROSCOPE:
            out << "IMU::Gyroscope";
            break;
        default:
            throw ImpossibleException("Unknown ImuDevice");
    }
    return out;
}
