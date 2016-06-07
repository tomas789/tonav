//
// Created by Tomas Krejci on 6/6/16.
//

#include "exceptions/general_exception.h"
#include "feature_track.h"

FeatureTrack::FeatureTrack(std::size_t first_frame_number) {
    first_frame_number_ = first_frame_number;
    is_out_of_view_ = false;
}

void FeatureTrack::addFeaturePosition(double x, double y) {
    Eigen::Vector2d position;
    position << x, y;
    positions_.push_back(std::move(position));
}

void FeatureTrack::revertLastPosition() {
    if (positions_.size() == 0) {
        throw GeneralException("Trying to revert last position of empty feature.");
    }
    positions_.pop_back();
}

std::size_t FeatureTrack::getFirstFrameNumber() const {
    return first_frame_number_;
}

bool FeatureTrack::isOutOfView() const {
    return is_out_of_view_;
}

void FeatureTrack::setOutOfView() {
    is_out_of_view_ = true;
}


