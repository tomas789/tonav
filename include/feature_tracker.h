//
// Created by Tomas Krejci on 5/30/16.
//

#ifndef TONAV_FEATURE_TRACKER_H
#define TONAV_FEATURE_TRACKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "frame_features.h"

class FeatureTracker {
public:
    FeatureTracker();

    void processImage(cv::Mat& image);

private:
    cv::Ptr<cv::Feature2D> detector_;
    cv::Ptr<cv::FlannBasedMatcher> matcher_;
    FrameFeatures previous_frame_features_;
};

#endif //TONAV_FEATURE_TRACKER_H
