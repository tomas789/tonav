//
// Created by Tomas Krejci on 5/30/16.
//

#include "feature_tracker.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>

#include "exceptions/general_exception.h"
#include "exceptions/impossible_exception.h"
#include "feature_track.h"

FeatureTracker::FeatureTracker() {
    detector_ = cv::FeatureDetector::create("ORB");
    extractor_ = cv::DescriptorExtractor::create("ORB");
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");

    frame_number_ = 0;
}

FeatureTracker::feature_track_list FeatureTracker::processImage(feature_track_list& previous_tracks, cv::Mat& image) {
    FrameFeatures frame_features = FrameFeatures::fromImage(detector_, extractor_, image);
    frame_features.drawFeatures(image);
    if (previous_frame_features_.keypoints().size() == 0) {
        FeatureTracker::feature_track_list current_features;
        for (std::size_t i = 0; i < frame_features.keypoints().size(); ++i) {
            const cv::KeyPoint& keypoint = frame_features.keypoints()[i];
            current_features.emplace_back(new FeatureTrack(frame_number_));
            current_features.back()->addFeaturePosition(keypoint.pt.x, keypoint.pt.y);
        }

        previous_frame_features_ = frame_features;
        frame_number_ += 1;
        return current_features;
    }

    std::vector<cv::DMatch> matches = frame_features.match(matcher_, previous_frame_features_);

    std::vector<double> previous_feature_matched(previous_tracks.size(), INFINITY);
    std::vector<bool> current_feature_matched(frame_features.keypoints().size(), false);
    feature_track_list current_tracks(frame_features.keypoints().size());

    double distance_limit = computeDistanceLimitForMatch(matches);

    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];

        if (match.distance > distance_limit) {
            continue;
        }

        int query_idx = match.queryIdx;
        int train_idx = match.trainIdx;

        if (current_feature_matched[query_idx]) {
            throw ImpossibleException("Current feature should match only once.");
        }

        if (match.distance < previous_feature_matched[train_idx]) {
            if (!std::isinf(previous_feature_matched[train_idx])) {
                // Revert previous match
                previous_tracks[train_idx]->revertLastPosition();
            }

            // Track feature
            const cv::KeyPoint& current_keypoint = frame_features.keypoints()[query_idx];
            current_tracks[query_idx] = previous_tracks[train_idx];
            current_tracks[query_idx]->addFeaturePosition(current_keypoint.pt.x, current_keypoint.pt.y);

            previous_feature_matched[train_idx] = match.distance;
            current_feature_matched[query_idx] = true;
        }
    }

    markOutOfViewFeatures(previous_feature_matched, previous_tracks);
    createNewFeatureTracks(current_feature_matched, current_tracks, frame_features);

    drawStats(image, previous_feature_matched, current_feature_matched, current_tracks, matches);

    previous_frame_features_ = frame_features;
    frame_number_ += 1;

    return current_tracks;
}

double FeatureTracker::computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const {
    double min_distance = 100;
    double max_distance = 0;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        if (match.distance < min_distance) {
            min_distance = match.distance;
        }
        if (match.distance > max_distance) {
            max_distance = match.distance;
        }
    }
    std::cout << "Min distance: " << min_distance << ", Max distance: " << max_distance << std::endl;

    return std::max(2*min_distance, 80.0);
}

void FeatureTracker::drawStats(cv::Mat &image, const std::vector<double> &previous_features_matched,
        const std::vector<bool> &current_features_matched, const feature_track_list& current_tracks,
        const std::vector<cv::DMatch>& matches) const {
    std::size_t out_of_view_features = 0;
    for (std::size_t i = 0; i < previous_features_matched.size(); ++i) {
        if (std::isinf(previous_features_matched[i])) {
            out_of_view_features += 1;
        }
    }

    std::size_t new_features = 0;
    for (std::size_t i = 0; i < current_features_matched.size(); ++i) {
        if (!current_features_matched[i]) {
            new_features += 1;
        }
    }

    double average_feature_live = 0.0;
    for (std::size_t i = 0; i < current_tracks.size(); ++i) {
        average_feature_live += (frame_number_ - current_tracks[i]->getFirstFrameNumber());
    }
    average_feature_live /= current_tracks.size();

    cv::putText(image, "Out-of-view features: " + std::to_string(out_of_view_features), cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    cv::putText(image, "New features: " + std::to_string(new_features), cv::Point(50, 65), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    cv::putText(image, "Matched features: " + std::to_string(current_features_matched.size() - new_features), cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    cv::putText(image, "Matches size: " + std::to_string(matches.size()), cv::Point(50, 95), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    cv::putText(image, "Average feature live: " + std::to_string(average_feature_live), cv::Point(50, 110), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));

}

void FeatureTracker::markOutOfViewFeatures(std::vector<double>& feature_matched, feature_track_list& feature_tracks)
        const {
    assert(feature_matched.size() == feature_tracks.size());

    for (std::size_t i = 0; i < feature_tracks.size(); ++i) {
        if (std::isinf(feature_matched[i])) {
            feature_tracks[i]->setOutOfView();
        }
    }
}

void FeatureTracker::createNewFeatureTracks(std::vector<bool> &feature_matched,
        feature_track_list &feature_tracks, const FrameFeatures& frame_features) const {
    for (std::size_t i = 0; i < feature_tracks.size(); ++i) {
        if (!feature_matched[i]) {
            // This is new feature
            const cv::KeyPoint& current_keypoint = frame_features.keypoints()[i];
            std::shared_ptr<FeatureTrack> feature_track(new FeatureTrack(frame_number_));
            feature_track->addFeaturePosition(current_keypoint.pt.x, current_keypoint.pt.y);
            feature_tracks[i] = feature_track;
        }
    }
}











