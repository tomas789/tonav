//
// Created by Tomas Krejci on 6/4/16.
//

#include "frame_features.h"

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "exceptions/general_exception.h"

FrameFeatures FrameFeatures::fromImage(cv::Ptr<cv::Feature2D> detector, cv::Mat& image) {
    FrameFeatures frame_features;

    cv::cvtColor(image, image, CV_GRAY2BGR);

    cv::Mat gray = FrameFeatures::toGray(image);
    frame_features.detectKeypoints(detector, gray);
    frame_features.computeDescriptors(detector, gray);

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


void FrameFeatures::detectKeypoints(cv::Ptr<cv::Feature2D> detector, cv::Mat gray) {
    int mid_x = gray.cols/2;
    int mid_y = gray.rows/2;
    for (int i = 0; i < 4; ++i) {
        int x = i % 2;
        int y = i / 2;

        std::vector<cv::KeyPoint> keypoints_part;
        cv::Mat image_part(gray, cv::Rect(x*mid_x, y*mid_y, mid_x, mid_y));
        detector->detect(image_part, keypoints_part);
        for (const cv::KeyPoint& kpt : keypoints_part) {
            cv::Point pt(x*mid_x + kpt.pt.x, y*mid_y + kpt.pt.y);
            keypoints_.emplace_back(pt, kpt.size, kpt.angle, kpt.response, kpt.octave, kpt.class_id);
        }
    }
}

void FrameFeatures::computeDescriptors(cv::Ptr<cv::Feature2D> detector, cv::Mat gray) {
    detector->compute(gray, keypoints_, descriptors_);
}

std::vector<cv::DMatch> FrameFeatures::match(cv::Ptr<cv::FlannBasedMatcher> matcher, const FrameFeatures &other) {
    if (other.descriptors_.rows == 0) {
        return std::vector<cv::DMatch>();
    }

    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_, other.descriptors_, matches);
    std::cout << "Matches: " << descriptors_.rows << std::endl;

    double min_dist = 100;
    double max_dist = 0;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        double distance = matches[i].distance;
        if (distance < min_dist) {
            min_dist = distance;
        }
        if (distance > max_dist) {
            max_dist = distance;
        }
    }

    double good_match_limit = std::max(2*min_dist, 0.02);
    std::vector<cv::DMatch> good_matches;
    for (std::size_t i = 0; i < matches.size(); ++i) {
        if (matches[i].distance < good_match_limit) {
            good_matches.push_back(matches[i]);
        }
    }

    return good_matches;
}





