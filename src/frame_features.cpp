//
// Created by Tomas Krejci on 6/4/16.
//

#include "frame_features.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "exceptions/general_exception.h"

FrameFeatures FrameFeatures::fromImage(cv::Ptr<cv::FeatureDetector> detector,
        cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat& image) {
    FrameFeatures frame_features;

    cv::cvtColor(image, image, CV_GRAY2BGR);

    cv::Mat gray = FrameFeatures::toGray(image);
    frame_features.detectKeypoints(detector, gray);
    frame_features.computeDescriptors(extractor, gray);

    frame_features.drawFeatures(image);

    return frame_features;
}

cv::Mat FrameFeatures::toGray(const cv::Mat& image) {
    switch (image.channels()) {
        case 3:
            {
                cv::Mat gray;
                cv::cvtColor(image, gray, CV_BGR2GRAY);
                return gray;
            }
        case 1:
            return image;
        default:
            throw GeneralException("Unknown image format");
    }
}

void FrameFeatures::drawFeatures(cv::Mat& image, cv::Scalar color) {
    int radius = 2;
    for (const cv::KeyPoint& kpt : keypoints_) {
        int x = kpt.pt.x;
        int y = kpt.pt.y;
        cv::line(image, cv::Point(x-radius, y), cv::Point(x+radius, y), color);
        cv::line(image, cv::Point(x, y-radius), cv::Point(x, y+radius), color);
        cv::rectangle(image, cv::Point(x-radius-2, y-radius-2), cv::Point(x+radius+2, y+radius+2), color);
    }
}


void FrameFeatures::detectKeypoints(cv::Ptr<cv::FeatureDetector> detector, cv::Mat gray) {
     detector->detect(gray, keypoints_);
//    exp_detector_.detect(gray, keypoints_);
}

void FrameFeatures::computeDescriptors(cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat gray) {
     extractor->compute(gray, keypoints_, descriptors_);
//    exp_extractor_.compute(gray, keypoints_, descriptors_);
}

std::vector<cv::DMatch> FrameFeatures::match(cv::Ptr<cv::DescriptorMatcher> matcher, const FrameFeatures &other, float threshold) {
    if (other.descriptors_.rows == 0) {
        return std::vector<cv::DMatch>();
    }
    
    bool use_ratio_test = false;
    if (use_ratio_test) {
        std::vector<std::vector<cv::DMatch>> matches;
        matcher->knnMatch(descriptors_, other.descriptors_, matches, 2);
        std::vector<cv::DMatch> good_matches;
        for (std::size_t i = 0; i < matches.size(); ++i) {
            std::vector<cv::DMatch>& feature_matches = matches[i];
            if (feature_matches.size() != 2) {
                std::cout << "Skipping feature " << i << " that has only " << feature_matches.size() << " matches" << std::endl;
                continue;
            }
            cv::DMatch better_match;
            cv::DMatch worser_match;
            if (feature_matches[0].distance < feature_matches[1].distance) {
                better_match = feature_matches[0];
                worser_match = feature_matches[1];
            } else {
                better_match = feature_matches[1];
                worser_match = feature_matches[0];
            }
            float ratio = better_match.distance / worser_match.distance;
            if (ratio < threshold) {
                if (better_match.distance < 15) {
                    good_matches.push_back(better_match);
                }
            }
            std::cout << "ratio " << ratio << std::endl;
        }
        return good_matches;
    } else {
        std::vector<cv::DMatch> matches;
        matcher->match(descriptors_, other.descriptors_, matches);
        
        double distance_limit = computeDistanceLimitForMatch(matches);
        std::vector<cv::DMatch> good_matches;
        for (std::size_t i = 0; i < matches.size(); ++i) {
            if (matches[i].distance < distance_limit) {
                good_matches.push_back(matches[i]);
            }
        }
        
        return good_matches;
    }
    
    
}

std::vector<cv::KeyPoint> &FrameFeatures::keypoints() {
    return keypoints_;
}

const std::vector<cv::KeyPoint> &FrameFeatures::keypoints() const {
    return keypoints_;
}

double FrameFeatures::computeDistanceLimitForMatch(const std::vector<cv::DMatch>& matches) const {
    double min_distance = 100;
    double max_distance = 0;
    double mean_distance = 0;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        const cv::DMatch& match = matches[i];
        mean_distance += match.distance;
        if (match.distance < min_distance) {
            min_distance = match.distance;
        }
        if (match.distance > max_distance) {
            max_distance = match.distance;
        }
    }
    mean_distance /= matches.size();
    std::cout << "Min distance: " << min_distance << ", Max distance: " << max_distance << ", mean distance: " << mean_distance << std::endl;
    
    return 5.0;
    return std::max(2*min_distance, 5.0);
}









