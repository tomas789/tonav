//
// Created by Tomas Krejci on 10/22/17.
//

#ifndef TONAV_GEOMETRY_H
#define TONAV_GEOMETRY_H

#include <Eigen/Core>
#include "quaternion.h"

namespace tonav {

class Geometry {
public:
    static Eigen::Vector3d switchFrames(const Eigen::Vector3d p_B_A, const tonav::Quaternion &q_B_A);
    
    static Eigen::Vector3d switchFrames(const Eigen::Vector3d p_B_A, const Eigen::Matrix3d &R_B_A);
};

}

#endif //TONAV_GEOMETRY_H
