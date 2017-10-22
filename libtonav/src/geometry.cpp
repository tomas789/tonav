//
// Created by Tomas Krejci on 10/22/17.
//

#include "geometry.h"

namespace tonav {

Eigen::Vector3d Geometry::switchFrames(const Eigen::Vector3d p_B_A, const tonav::Quaternion &q_B_A) {
    return Geometry::switchFrames(p_B_A, q_B_A.toRotationMatrix());
}

Eigen::Vector3d Geometry::switchFrames(const Eigen::Vector3d p_B_A, const Eigen::Matrix3d &R_B_A) {
    return -R_B_A * p_B_A;
}

}
