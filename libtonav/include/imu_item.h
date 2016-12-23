//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_IMU_ITEM_H
#define TONAV_IMU_ITEM_H

#include <string>
#include <Eigen/Core>

#include "imu_device.h"

class ImuBuffer;

class ImuItem {
public:
    friend class ImuBuffer;
    
    static ImuItem fromVector3d(double time, const ImuDevice& device, const Eigen::Vector3d& data);

    ImuDevice getDevice() const;

    double getTime() const;

    double getX() const;
    double getY() const;
    double getZ() const;
    Eigen::Vector3d getVector() const;
private:
    ImuDevice device_;
    double time_;
    Eigen::Vector3d data_;
};

#endif //TONAV_IMU_ITEM_H
