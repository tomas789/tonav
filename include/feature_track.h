//
// Created by Tomas Krejci on 6/6/16.
//

#ifndef TONAV_FEATURE_TRACK_H
#define TONAV_FEATURE_TRACK_H

#include <Eigen/Dense>
#include <vector>

class FeatureTrack {
public:
    FeatureTrack();

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
    int getFeatureId() const;
    
private:
    int feature_id_;
    bool is_out_of_view_;
    bool was_used_for_residualization_;
    std::vector<Eigen::Vector2d> positions_;
};

#endif //TONAV_FEATURE_TRACK_H
