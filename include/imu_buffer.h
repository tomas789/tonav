//
// Created by Tomas Krejci on 5/11/16.
//

#ifndef TONAV_IMU_BUFFER_H
#define TONAV_IMU_BUFFER_H

#include <vector>

#include "imu_device.h"
#include "imu_item.h"

/**
 * @brief Buffer of IMU data measurements.
 *
 * @deprecated This class is no longer needed. It is used in Navigator
 *             class only.
 *
 * Using this class you can interpolate IMU measurement at any time in
 * interval [getMinTime(), getMaxTime()]
 */
class ImuBuffer {
public:
    ImuBuffer(ImuDevice device, std::size_t size);

    void addMeasurement(ImuItem item);
    ImuItem interpolateAtTime(double time) const;
    double getMinTime() const;
    double getMaxTime() const;

    bool isReady() const;
private:
    ImuDevice device_;
    std::size_t size_;
    std::size_t cur_pos_;
    std::vector<ImuItem> items_;
};

#endif //TONAV_IMU_BUFFER_H
