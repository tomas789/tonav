//
// Created by Tomas Krejci on 6/4/16.
//

#ifndef TONAV_FRAME_FEATURES_H
#define TONAV_FRAME_FEATURES_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class FrameFeatures {
public:
    static FrameFeatures fromImage(cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> extractor, cv::Mat& image);

    std::vector<cv::DMatch> match(cv::Ptr<cv::DescriptorMatcher> matcher, const FrameFeatures& other);
    void drawFeatures(cv::Mat& image, cv::Scalar color = cv::Scalar(255, 0, 0));

    std::vector<cv::KeyPoint>& keypoints();
    const std::vector<cv::KeyPoint>& keypoints() const;

private:
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;

    void detectKeypoints(cv::Ptr<cv::FeatureDetector> detector, cv::Mat gray);
    void computeDescriptors(cv::Ptr<cv::DescriptorExtractor> detector, cv::Mat gray);

    static cv::Mat toGray(cv::Mat image);
};

#endif //TONAV_FRAME_FEATURES_H
