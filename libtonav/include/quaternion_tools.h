//
// Created by Tomas Krejci on 11/17/16.
//

#ifndef TONAV_QUATERNION_TOOLS_H
#define TONAV_QUATERNION_TOOLS_H

#include <Eigen/Core>

class QuaternionTools {
public:
    static Eigen::Matrix3d crossMatrix(const Eigen::Vector3d& v);
    static Eigen::Matrix4d bigOmegaMatrix(const Eigen::Vector3d& v);
};

#endif //TONAV_QUATERNION_TOOLS_H
