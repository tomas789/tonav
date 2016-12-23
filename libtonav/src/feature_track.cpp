//
// Created by Tomas Krejci on 6/6/16.
//

#include "feature_track.h"

#include <opencv2/core/core.hpp>

#include "exceptions/general_exception.h"

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

void FeatureTrack::drawFeatureTrack(cv::Mat& image, cv::Scalar color, int thickness) const {
    if (positions_.size() < 2) {
        return;
    }
    
    for (std::size_t i = 1; i < positions_.size(); ++i) {
        const Eigen::Vector2d& from = positions_[i-1];
        const Eigen::Vector2d& to = positions_[i];
        cv::line(image, cv::Point(from(0), from(1)), cv::Point(to(0), to(1)), color, thickness);
    }
}
