//
// Created by Tomas Krejci on 5/10/16.
//

#include "calibration.h"
#include "imu_item.h"

namespace tonav {

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

}
