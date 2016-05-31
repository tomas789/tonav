//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_IMU_ITEM_H
#define TONAV_IMU_ITEM_H

#include <string>

#include "imu_device.h"

class ImuBuffer;

class ImuItem {
public:
    friend class ImuBuffer;
    static ImuItem fromString(std::string line);

    ImuDevice getDevice() const;

    double getTime() const;

    double getX() const;
    double getY() const;
    double getZ() const;
private:
    ImuDevice device_;
    double time_;
    double x_;
    double y_;
    double z_;
};

#endif //TONAV_IMU_ITEM_H
