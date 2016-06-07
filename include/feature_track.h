//
// Created by Tomas Krejci on 6/6/16.
//

#ifndef TONAV_FEATURE_TRACK_H
#define TONAV_FEATURE_TRACK_H

#include <Eigen/Dense>
#include <vector>

class FeatureTrack {
public:
    FeatureTrack(std::size_t first_frame_number);

    void addFeaturePosition(double x, double y);
    void revertLastPosition();

    std::size_t getFirstFrameNumber() const;

    bool isOutOfView() const;
    void setOutOfView();
private:
    bool is_out_of_view_;
    std::size_t first_frame_number_;
    std::vector<Eigen::Vector2d> positions_;
};

#endif //TONAV_FEATURE_TRACK_H
