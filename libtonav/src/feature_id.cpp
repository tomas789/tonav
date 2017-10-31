//
// Created by Tomas Krejci on 10/31/17.
//

#include "feature_id.h"

namespace tonav {

FeatureId::FeatureId(std::size_t frame_id, std::size_t feature_id)
    : frame_id_(frame_id),
      feature_id_(feature_id)
{

}

bool FeatureId::operator==(const FeatureId& other) const {
    return frame_id_ == other.frame_id_ && feature_id_ == other.feature_id_;
}

bool FeatureId::operator<(const FeatureId& other) const {
    if (frame_id_ != other.frame_id_) {
        return frame_id_ < other.frame_id_;
    } else {
        return feature_id_ < other.feature_id_;
    }
}

std::size_t FeatureId::getFeatureId() const {
    return feature_id_;
}

std::size_t FeatureId::getFrameId() const {
    return frame_id_;
}

} // namespace tonav
