//
// Created by Tomas Krejci on 10/31/17.
//

#ifndef TONAV_FEATURE_ID_H
#define TONAV_FEATURE_ID_H

#include <utility>

namespace tonav {

class FeatureId {
public:
    FeatureId(std::size_t frame_id, std::size_t feature_id);
    
    bool operator==(const FeatureId& other) const;
    bool operator<(const FeatureId& other) const;
    
    std::size_t getFeatureId() const;
    std::size_t getFrameId() const;
    
private:
    std::size_t frame_id_;
    std::size_t feature_id_;
};

}

#endif //TONAV_FEATURE_ID_H
