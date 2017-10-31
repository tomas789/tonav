//
// Created by Tomas Krejci on 6/6/16.
//

#ifndef TONAV_FEATURE_TRACK_H
#define TONAV_FEATURE_TRACK_H

#include <Eigen/Core>
#include <vector>
#include <opencv2/core/core.hpp>

#include "feature_id.h"

namespace tonav {

class FeatureTrack {
public:
    FeatureTrack(const FeatureId& feature_id);
    
    const Eigen::Vector2d &operator[](std::size_t i) const;
    
    void addFeaturePosition(double x, double y);
    
    void revertLastPosition();
    
    std::size_t posesTrackedCount() const;
    
    bool isOutOfView() const;
    
    void setOutOfView();
    
    bool wasUsedForResidualization() const;
    
    void setWasUsedForResidualization();
    
    /**
     * @brief For debugging purpose only.
     *
     * @todo This should be deleted in the future. I will keep it here just for case ...
     */
    const FeatureId& getFeatureId() const;
    
    void drawFeatureTrack(cv::Mat &image, cv::Scalar color, int thickness) const;

private:
    FeatureId feature_id_;
    bool is_out_of_view_;
    bool was_used_for_residualization_;
    std::vector<Eigen::Vector2d> positions_;
};

}

#endif //TONAV_FEATURE_TRACK_H
