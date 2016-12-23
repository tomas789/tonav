//
// Created by Tomas Krejci on 5/30/16.
//

#ifndef TONAV_FEATURE_TRACKER_H
#define TONAV_FEATURE_TRACKER_H

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "frame_features.h"
#include "feature_track.h"

class Filter;

class FeatureTracker {
public:
    friend class Filter;
    
    using feature_track_list = std::vector<std::shared_ptr<FeatureTrack>>;
    
    FeatureTracker(int nfeatures_to_track);
    
    feature_track_list processImage(feature_track_list& previous_tracks, cv::Mat& image);

private:
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    FrameFeatures previous_frame_features_;

    double computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const;
    void drawStats(cv::Mat& image, const std::vector<double>& previous_features_matched,
            const std::vector<bool>& current_features_matched, const feature_track_list& current_tracks,
            const std::vector<cv::DMatch>& matches) const;
    void markOutOfViewFeatures(std::vector<double>& feature_matched, feature_track_list& feature_tracks) const;
    void createNewFeatureTracks(std::vector<bool>& feature_matched, feature_track_list& feature_tracks,
        const FrameFeatures& frame_features, double scale_factor) const;
};


#endif //TONAV_FEATURE_TRACKER_H
