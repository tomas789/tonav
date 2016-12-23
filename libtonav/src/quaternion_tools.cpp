//
// Created by Tomas Krejci on 11/17/16.
//

#include "quaternion_tools.h"

Eigen::Matrix3d QuaternionTools::crossMatrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d m;
    m << 0, -1*v(2), v(1),
         v(2), 0, -1*v(0),
         -1*v(1), v(0), 0;
    return m;
}

Eigen::Matrix4d QuaternionTools::bigOmegaMatrix(const Eigen::Vector3d &v) {
    Eigen::Matrix4d m;
    m(3, 3) = 0.0;
    m.block<3, 3>(0, 0) = -1*QuaternionTools::crossMatrix(v);
    m.block<3, 1>(0, 3) = v;
    m.block<1, 3>(3, 0) = -1*v.transpose();
    return m;
}
