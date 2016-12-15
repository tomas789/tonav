//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_CAMERA_ITEM_H
#define TONAV_CAMERA_ITEM_H

#include <string>
#include <opencv2/core/core.hpp>

class CameraItem {
public:
    CameraItem();
    CameraItem(double time, const cv::Mat& image);
    CameraItem(const CameraItem& other) = default;
    
    CameraItem& operator=(const CameraItem& oher) = default;
    
    operator bool() const;
    
    void setIsProcessed();
    bool wasProcessed() const;

    double getTime() const;
    cv::Mat& getImage();
    const cv::Mat& cgetImage() const;

private:
    bool is_valid_;
    double time_;
    cv::Mat image_;
    bool was_processed_;
};

#endif //TONAV_CAMERA_ITEM_H
