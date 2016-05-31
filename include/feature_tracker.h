//
// Created by Tomas Krejci on 5/30/16.
//

#ifndef TONAV_FEATURE_TRACKER_H
#define TONAV_FEATURE_TRACKER_H

#include <opencv2/core/core.hpp>

class FeatureTracker {
public:
    void processImage(cv::Mat image);
};

#endif //TONAV_FEATURE_TRACKER_H
