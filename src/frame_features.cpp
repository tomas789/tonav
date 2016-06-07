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

cv::Mat FrameFeatures::toGray(cv::Mat image) {
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
}

void FrameFeatures::computeDescriptors(cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat gray) {
    extractor->compute(gray, keypoints_, descriptors_);
}

std::vector<cv::DMatch> FrameFeatures::match(cv::Ptr<cv::DescriptorMatcher> matcher, const FrameFeatures &other) {
    if (other.descriptors_.rows == 0) {
        return std::vector<cv::DMatch>();
    }

    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_, other.descriptors_, matches);
    return matches;
}

std::vector<cv::KeyPoint> &FrameFeatures::keypoints() {
    return keypoints_;
}

const std::vector<cv::KeyPoint> &FrameFeatures::keypoints() const {
    return keypoints_;
}









