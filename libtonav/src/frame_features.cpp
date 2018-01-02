//
// Created by Tomas Krejci on 6/4/16.
//

#include "frame_features.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "exceptions/general_exception.h"

namespace tonav {

std::size_t FrameFeatures::frame_counter_;

std::shared_ptr<FrameFeatures> FrameFeatures::fromImage(
    cv::Ptr<cv::FeatureDetector> detector,
    cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat &image
) {
    std::shared_ptr<FrameFeatures> frame_features(new FrameFeatures);
    
    assert(image.channels() == 3);
    
    cv::Mat gray = FrameFeatures::toGray(image);
    if (detector.get() == extractor.get()) {
        detector->detectAndCompute(
            gray,
            cv::Mat(),
            frame_features->keypoints_,
            frame_features->descriptors_
        );
    } else {
        frame_features->detectKeypoints(detector, gray);
        frame_features->computeDescriptors(extractor, gray);
    }
    
    for (int i = 0; i < frame_features->keypoints_.size(); ++i) {
        frame_features->feature_ids_.emplace_back(frame_features->frame_id_, i);
    }
    
    frame_features->drawFeatures(image);
    
    return frame_features;
}

cv::Mat FrameFeatures::toGray(const cv::Mat &image) {
    switch (image.channels()) {
        case 1:
            return image;
        case 3: {
            cv::Mat gray;
            cv::cvtColor(image, gray, CV_BGR2GRAY);
            return gray;
        }
        default:
            throw GeneralException("Unknown image format");
    }
}

void FrameFeatures::drawFeatures(cv::Mat &image, cv::Scalar color, double scale_factor) {
    int radius = 2;
    for (const cv::KeyPoint &kpt : keypoints_) {
        int x = kpt.pt.x / scale_factor;
        int y = kpt.pt.y / scale_factor;
        cv::line(image, cv::Point(x - radius, y), cv::Point(x + radius, y), color);
        cv::line(image, cv::Point(x, y - radius), cv::Point(x, y + radius), color);
        cv::rectangle(image, cv::Point(x - radius - 2, y - radius - 2), cv::Point(x + radius + 2, y + radius + 2), color);
    }
}


void FrameFeatures::detectKeypoints(cv::Ptr<cv::FeatureDetector> detector, cv::Mat gray) {
    detector->detect(gray, keypoints_);
}

void FrameFeatures::computeDescriptors(cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat gray) {
    extractor->compute(gray, keypoints_, descriptors_);
}

std::vector<cv::DMatch> FrameFeatures::match(cv::Ptr<cv::DescriptorMatcher> matcher, const FrameFeatures &other, float threshold) {
    if (other.descriptors_.rows == 0) {
        return std::vector<cv::DMatch>();
    }
    
    std::vector<std::vector<cv::DMatch>> matches;
    matcher->knnMatch(descriptors_, other.descriptors_, matches, 2);
    
    std::vector<cv::DMatch> good_matches;
    for (int i = 0; i < matches.size(); ++i) {
        if (matches[i][0].distance <= 0.7*matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }
    
    bool use_fundamental_filter = true;
    if (use_fundamental_filter) {
        std::vector<cv::Point2f> this_pts;
        std::vector<cv::Point2f> other_pts;
        this_pts.reserve(good_matches.size());
        other_pts.reserve(good_matches.size());
        for (std::size_t i = 0; i < good_matches.size(); ++i) {
            this_pts.push_back(keypoints_[good_matches[i].queryIdx].pt);
            other_pts.push_back(other.keypoints_[good_matches[i].trainIdx].pt);
        }
        cv::Mat good_matches_mask;
        cv::Mat F = cv::findFundamentalMat(this_pts, other_pts, CV_FM_RANSAC, 3, 0.99, good_matches_mask);
        
        std::vector<cv::DMatch> good_matches_filtered;
        for (std::size_t i = 0; i < good_matches.size(); ++i) {
            if (good_matches_mask.at<bool>(i, 0)) {
                good_matches_filtered.push_back(good_matches[i]);
            }
        }
        return good_matches_filtered;
    } else {
        return good_matches;
    }
}

std::vector<cv::KeyPoint> &FrameFeatures::keypoints() {
    return keypoints_;
}

const std::vector<cv::KeyPoint> &FrameFeatures::keypoints() const {
    return keypoints_;
}

const cv::Mat& FrameFeatures::descriptors() const {
    return descriptors_;
}

const std::vector<FeatureId>& FrameFeatures::getFeatureIds() const {
    return feature_ids_;
}

std::size_t FrameFeatures::getFrameId() const {
    return frame_id_;
}

double FrameFeatures::computeDistanceLimitForMatch(const std::vector<cv::DMatch> &matches) const {
    double min_distance = 100;
    double max_distance = 0;
    double mean_distance = 0;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch &match = matches[i];
        mean_distance += match.distance;
        if (match.distance < min_distance) {
            min_distance = match.distance;
        }
        if (match.distance > max_distance) {
            max_distance = match.distance;
        }
    }
    mean_distance /= matches.size();
    return std::max(2 * min_distance, 5.0);
}

FrameFeatures::FrameFeatures()
    : frame_id_(FrameFeatures::frame_counter_++)
{
}

} // namespace tonav
