#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "quaternion.h"

TEST(Quaternion, test_identity) {
    Quaternion q = Quaternion::identity();
    ASSERT_EQ(q.x(), 0);
    ASSERT_EQ(q.y(), 0);
    ASSERT_EQ(q.z(), 0);
    ASSERT_EQ(q.w(), 1);
}

TEST(Quaternion, test_conjugate) {
    Quaternion q(1.23, M_PI, M_E, M_LN10); // random
    q = q.conjugate();
    ASSERT_EQ(q.x(), -1.23);
    ASSERT_EQ(q.y(), -M_PI);
    ASSERT_EQ(q.z(), -M_E);
    ASSERT_EQ(q.w(), M_LN10);
}

TEST(Quaternion, test_multiplication_with_conjugate) {
    Quaternion q(1.23, M_PI, M_E, M_LN10); // random
    q.normalize();

    Quaternion q_id = q * q.conjugate();
    ASSERT_TRUE(q_id.isApprox(Quaternion::identity(), 1e-5));
}

TEST(Quaternion, test_to_rotation_matrix_and_back) {
    Eigen::Quaterniond q_orig(1.23, M_PI, 0.28*M_E, 0.3*M_LN10); // random
    q_orig.normalize();
    Eigen::Matrix3d r = q_orig.toRotationMatrix();

    Quaternion q = Quaternion::fromRotationMatrix(r);
    Eigen::Matrix3d r_converted = q.toRotationMatrix();

    ASSERT_TRUE((r - r_converted).isZero(1e-12));
}
