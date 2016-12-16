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
