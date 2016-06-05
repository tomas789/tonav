//
// Created by Tomas Krejci on 5/30/16.
//

#include "feature_tracker.h"

#include <opencv2/core/core.hpp>

#include "exceptions/general_exception.h"

FeatureTracker::FeatureTracker() {
    detector_ = cv::Ptr<cv::Feature2D>(new cv::ORB(300));
    matcher_ = cv::Ptr<cv::FlannBasedMatcher>(new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(5, 24, 2)));
}

void FeatureTracker::processImage(cv::Mat& image) {
    FrameFeatures frame_features = FrameFeatures::fromImage(detector_, image);
    frame_features.drawFeatures(image);

    std::vector<cv::DMatch> matches = frame_features.match(matcher_, previous_frame_features_);

    previous_frame_features_ = frame_features;
}



