#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "quaternion.h"
#include "quaternion_tools.h"

TEST(QuaternionTools, test_cross_matrix) {
    // Indirect Kalman Filter for 3D Attitude Estimation, Eqn. (78)

    Quaternion q(1.0, 2.0, 3.0, M_PI); // random
    q.normalize();
    
    Eigen::Matrix3d r = q.toRotationMatrix();
    
    auto p1 = (2*q.w()*q.w() - 1)*Eigen::Matrix3d::Identity();
    auto p2 = -2*q.w()*QuaternionTools::crossMatrix(q.vec());
    auto p3 = 2*q.vec()*q.vec().transpose();
    auto r_from_q = p1 + p2 + p3;
    
    ASSERT_TRUE(r.isApprox(r_from_q));
}

TEST(QuaternionTools, test_big_omega_matrix) {
    // Indirect Kalman Filter for 3D Attitude Estimation, Eqn. (105,106)

    Quaternion q(1.0, 2.0, 3.0, M_PI); // random
    q.normalize();
    
    Eigen::Vector3d perturb_vec;
    perturb_vec << 0.001, 0.02, -0.004;
    perturb_vec.normalize();
    
    Eigen::Vector4d q_as_vector;
    q_as_vector << q.x(), q.y(), q.z(), q.w();
    Eigen::Vector4d q_dot_vec = 0.5 * QuaternionTools::bigOmegaMatrix(perturb_vec) * q_as_vector;
    Quaternion q_lhs(q_dot_vec(0), q_dot_vec(1), q_dot_vec(2), q_dot_vec(3));
    q_lhs.normalize();
    
    Quaternion q_perturb(perturb_vec(0), perturb_vec(1), perturb_vec(2), 0.0);
    Quaternion q_rhs = Quaternion(0.0, 0.0, 0.0, 0.5) * q_perturb * q;
    q_rhs.normalize();

    ASSERT_TRUE(q_lhs.isApprox(q_rhs, 1e-3));
}
