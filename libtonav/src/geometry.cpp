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

Eigen::Vector3d Geometry::transformFrames(const Eigen::Vector3d p_x_B, const tonav::Quaternion& q_A_B, const Eigen::Vector3d p_A_B) {
    return Geometry::transformFrames(p_x_B, q_A_B.toRotationMatrix(), p_A_B);
}

Eigen::Vector3d Geometry::transformFrames(const Eigen::Vector3d p_x_B, const Eigen::Matrix3d& R_A_B, const Eigen::Vector3d p_A_B) {
    return R_A_B*(p_x_B - p_A_B);
}

}
