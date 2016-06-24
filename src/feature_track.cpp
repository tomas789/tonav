//
// Created by Tomas Krejci on 6/6/16.
//

#include "exceptions/general_exception.h"
#include "feature_track.h"

FeatureTrack::FeatureTrack() {
    static int feature_id = 0;
    feature_id_ = feature_id++;
    
    is_out_of_view_ = false;
    was_used_for_residualization_ = false;
}

const Eigen::Vector2d& FeatureTrack::operator[](std::size_t i) const {
    assert(i < positions_.size());
    return positions_[i];
}

void FeatureTrack::addFeaturePosition(double x, double y) {
    Eigen::Vector2d position;
    position << x, y;
    positions_.push_back(std::move(position));
}

void FeatureTrack::revertLastPosition() {
    assert(positions_.size() > 0);
    positions_.pop_back();
}

std::size_t FeatureTrack::posesTrackedCount() const {
    return positions_.size();
}

bool FeatureTrack::isOutOfView() const {
    return is_out_of_view_;
}

void FeatureTrack::setOutOfView() {
    is_out_of_view_ = true;
}

bool FeatureTrack::wasUsedForResidualization() const {
    return was_used_for_residualization_;
}

void FeatureTrack::setWasUsedForResidualization() {
    assert(!was_used_for_residualization_);
    was_used_for_residualization_ = true;
}

int FeatureTrack::getFeatureId() const {
    return feature_id_;
}
