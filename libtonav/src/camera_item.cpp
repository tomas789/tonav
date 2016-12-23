//
// Created by Tomas Krejci on 5/11/16.
//

#include "camera_item.h"

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "calibration.h"
#include "exceptions/general_exception.h"

CameraItem::CameraItem() : is_valid_(false) {
}

CameraItem::CameraItem(double time, const cv::Mat& image) : is_valid_(true) {
    time_ = time;
    if (image.channels() == 1) {
        cv::cvtColor(image, image_, CV_GRAY2BGR);
    } else {
        image_ = image;
    }
    was_processed_ = false;
}

CameraItem::operator bool() const {
    return is_valid_;
}

void CameraItem::setIsProcessed() {
    assert(was_processed_ == false);
    was_processed_ = true;
}

bool CameraItem::wasProcessed() const {
    return was_processed_;
}

double CameraItem::getTime() const {
    return time_;
}

cv::Mat& CameraItem::getImage() {
    return image_;
}

const cv::Mat& CameraItem::cgetImage() const {
    return image_;
}
