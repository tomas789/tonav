//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_IMU_DEVICE_H
#define TONAV_IMU_DEVICE_H

#include <iostream>

enum class ImuDevice {
    ACCELEROMETER, GYROSCOPE
};

std::ostream& operator<<(std::ostream& out, const ImuDevice& device);

#endif //TONAV_IMU_DEVICE_H
