//
// Created by Tomas Krejci on 6/4/16.
//

#ifndef TONAV_FRAME_FEATURES_H
#define TONAV_FRAME_FEATURES_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <memory>
#include <vector>

#include "feature_id.h"

namespace tonav {

class FrameFeatures {
public:
    static std::shared_ptr<FrameFeatures> fromImage(
        cv::Ptr<cv::FeatureDetector> detector, cv::Ptr<cv::DescriptorExtractor> extractor,
        cv::Mat &image
    );
    
    /**
     * @brief Match features to other frame
     *
     * @param matcher Matcher to match with.
     * @param other FrameFeatures instance representing older frame
     * @param threshold Feature quality threshold. Smaller means more strict.
     * @return Matches found.
     */
    std::vector<cv::DMatch> match(
        cv::Ptr<cv::DescriptorMatcher> matcher, const FrameFeatures &other,
        float threshold = 0.5
    );
    
    void drawFeatures(cv::Mat &image, cv::Scalar color = cv::Scalar(255, 0, 0), double scale_factor = 1.0);
    
    std::vector<cv::KeyPoint> &keypoints();
    
    const std::vector<cv::KeyPoint> &keypoints() const;
    
    const cv::Mat& descriptors() const;
    
    const std::vector<FeatureId>& getFeatureIds() const;
    
    std::size_t getFrameId() const;
    
    double computeDistanceLimitForMatch(const std::vector<cv::DMatch> &matches) const;
    
protected:
    std::size_t frame_id_;
    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;
    std::vector<FeatureId> feature_ids_;
    
    FrameFeatures();
    
    static std::size_t frame_counter_;
    
    void detectKeypoints(cv::Ptr<cv::FeatureDetector> detector, cv::Mat gray);
    
    void computeDescriptors(cv::Ptr<cv::DescriptorExtractor> detector, cv::Mat gray);
    
    static cv::Mat toGray(const cv::Mat &image);
};

} // namespace tonav

#endif //TONAV_FRAME_FEATURES_H
