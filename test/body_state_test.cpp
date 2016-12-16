#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>

#include "body_state.h"
#include "calibration.h"
#include "quaternion.h"

class MockCalibration : public Calibration {
public:
    friend class BodyStateNoMovementTest;
    friend class BodyStatePathTrajectoryTest;
};

class BodyStateNoMovementTest : public ::testing::Test {
public:
    std::shared_ptr<MockCalibration> calibration_;
    std::shared_ptr<BodyState> body_state_;
    
    BodyStateNoMovementTest() {
        calibration_ = std::shared_ptr<MockCalibration>(new MockCalibration);
        Eigen::Vector3d global_gravity;
        global_gravity << 0.0, 0.0, -9.81;
        calibration_->global_gravity_ = global_gravity;
    }
    
    void apply_no_movement() {
        Eigen::Vector3d rotation_estimate;
        rotation_estimate << 0.0, 0.0, 0.0;
        Eigen::Vector3d acceleration_estimate;
        acceleration_estimate << 0.0, 0.0, 0.0;
        Quaternion q_B_G = Quaternion::identity();
        Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_B_G = Eigen::Vector3d::Zero();
        body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
    }
    
    void apply_no_movement_one_step() {
        Eigen::Vector3d rotation_estimate;
        rotation_estimate << 0.0, 0.0, 0.0;
        Eigen::Vector3d acceleration_estimate;
        acceleration_estimate << 0.0, 0.0, 9.81;
        Quaternion q_B_G = Quaternion::identity();
        Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_B_G = Eigen::Vector3d::Zero();
        body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
        body_state_ = BodyState::propagate(*body_state_, 1e-2, rotation_estimate, acceleration_estimate);
    }
    
    void apply_no_movement_for_one_sec_at_100hz() {
        Eigen::Vector3d rotation_estimate;
        rotation_estimate << 0.0, 0.0, 0.0;
        Eigen::Vector3d acceleration_estimate;
        acceleration_estimate << 0.0, 0.0, 9.81;
        Quaternion q_B_G = Quaternion::identity();
        Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_B_G = Eigen::Vector3d::Zero();
        body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
        
        for (int i = 1; i < 101; ++i) {
            body_state_ = BodyState::propagate(*body_state_, i/100.0, rotation_estimate, acceleration_estimate);
        }
    }
    
    void apply_free_fall_for_one_sec_at_100hz() {
        Eigen::Vector3d rotation_estimate;
        rotation_estimate << 0.0, 0.0, 0.0;
        Eigen::Vector3d acceleration_estimate;
        acceleration_estimate << 0.0, 0.0, 0.0;
        Quaternion q_B_G = Quaternion::identity();
        Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_B_G = Eigen::Vector3d::Zero();
        body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
        
        for (int i = 1; i < 101; ++i) {
            body_state_ = BodyState::propagate(*body_state_, i/100.0, rotation_estimate, acceleration_estimate);
        }
    }
    
    void apply_move_forward_for_one_sec_at_100hz() {
        Eigen::Vector3d rotation_estimate;
        rotation_estimate << 0.0, 0.0, 0.0;
        Eigen::Vector3d acceleration_estimate;
        acceleration_estimate << 1.0, 0.0, 9.81;
        Quaternion q_B_G = Quaternion::identity();
        Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_B_G = Eigen::Vector3d::Zero();
        body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
        
        for (int i = 1; i < 101; ++i) {
            body_state_ = BodyState::propagate(*body_state_, i/100.0, rotation_estimate, acceleration_estimate);
        }
    }
    
    void apply_rotate_clockwise_for_one_sec_at_100hz() {
        Eigen::Vector3d rotation_estimate;
        rotation_estimate << 0.0, 0.0, -0.5*M_PI;
        Eigen::Vector3d acceleration_estimate;
        acceleration_estimate << 0.0, 0.0, 9.81;
        Quaternion q_B_G = Quaternion::identity();
        Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_B_G = Eigen::Vector3d::Zero();
        body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
        
        for (int i = 1; i < 101; ++i) {
            body_state_ = BodyState::propagate(*body_state_, i/100.0, rotation_estimate, acceleration_estimate);
        }
    }
    
    void apply_rotate_and_go() {
        Eigen::Vector3d rotation_estimate;
        rotation_estimate << 0.0, 0.0, -0.5*M_PI;
        Eigen::Vector3d acceleration_estimate;
        acceleration_estimate << 0.0, 0.0, 9.81;
        Quaternion q_B_G = Quaternion::identity();
        Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_B_G = Eigen::Vector3d::Zero();
        body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G);
        
        for (int i = 1; i < 101; ++i) {
            body_state_ = BodyState::propagate(*body_state_, i/100.0, rotation_estimate, acceleration_estimate);
        }
        
        Eigen::Vector3d rotation_estimate_forward;
        rotation_estimate_forward << 0.0, 0.0, 0.0;
        Eigen::Vector3d acceleration_estimate_forward;
        acceleration_estimate_forward << 1.0, 0.0, 9.81;
        body_state_ = BodyState::propagate(*body_state_, 1 + 1e-12, rotation_estimate_forward, acceleration_estimate_forward);
        for (int i = 1; i < 101; ++i) {
            body_state_ = BodyState::propagate(*body_state_, 1 + i/100.0, rotation_estimate_forward, acceleration_estimate_forward);
        }
    }
};

TEST_F(BodyStateNoMovementTest, test_global_gravity) {
    Eigen::Vector3d global_gravity;
    global_gravity << 0.0, 0.0, -9.81;
    auto result = calibration_->getGlobalGravity();
    ASSERT_EQ(result, global_gravity) << "Got global gravity of [" << result.transpose() << "]^T";
}

TEST_F(BodyStateNoMovementTest, test_no_movement_time_is_zero) {
    apply_no_movement();
    ASSERT_EQ(body_state_->time(), 0.0);
    ASSERT_TRUE(body_state_->getOrientationInGlobalFrame().isApprox(Quaternion::identity())) << "Orientation should be identity";
    ASSERT_TRUE(body_state_->getPositionInGlobalFrame().isZero(1e-12));
    ASSERT_TRUE(body_state_->getVelocityInGlobalFrame().isZero(1e-12));
}

TEST_F(BodyStateNoMovementTest, test_no_movement_one_step) {
    apply_no_movement_one_step();
    ASSERT_EQ(body_state_->time(), 1e-2);
    ASSERT_TRUE(body_state_->getOrientationInGlobalFrame().isApprox(Quaternion::identity())) << "Orientation should be identity";
    auto position = body_state_->getPositionInGlobalFrame();
    ASSERT_TRUE(position.isZero(1e-12)) << "Got position [" << position.transpose() << "]^T";
    auto velocity = body_state_->getVelocityInGlobalFrame();
    ASSERT_TRUE(velocity.isZero(1e-12)) << "Got velocity [" << velocity.transpose() << "]^T";
}

TEST_F(BodyStateNoMovementTest, test_free_fall_for_one_sec_at_100hz) {
    apply_free_fall_for_one_sec_at_100hz();
    ASSERT_EQ(body_state_->time(), 1.0);
    ASSERT_TRUE(body_state_->getOrientationInGlobalFrame().isApprox(Quaternion::identity())) << "Orientation should be identity";
    
    Eigen::Vector3d position;
    position << 0.0, 0.0, -9.81/2;
    auto position_result = body_state_->getPositionInGlobalFrame();
    ASSERT_TRUE(position_result.isApprox(position)) << "Got position_result [" << position_result.transpose() << "]^T";
    
    Eigen::Vector3d velocity;
    velocity << 0.0, 0.0, -9.81;
    auto velocity_result = body_state_->getVelocityInGlobalFrame();
    ASSERT_TRUE(velocity_result.isApprox(velocity)) << "Got velocity_result [" << velocity_result.transpose() << "]^T";
}

TEST_F(BodyStateNoMovementTest, test_no_movement_for_one_sec_at_100hz) {
    apply_no_movement_for_one_sec_at_100hz();
    ASSERT_EQ(body_state_->time(), 1.0);
    ASSERT_TRUE(body_state_->getOrientationInGlobalFrame().isApprox(Quaternion::identity())) << "Orientation should be identity";
    auto position = body_state_->getPositionInGlobalFrame();
    ASSERT_TRUE(position.isZero(1e-12)) << "Got position [" << position.transpose() << "]^T";
    auto velocity = body_state_->getVelocityInGlobalFrame();
    ASSERT_TRUE(velocity.isZero(1e-12)) << "Got velocity [" << velocity.transpose() << "]^T";
}

TEST_F(BodyStateNoMovementTest, test_move_forward_for_one_sec_at_100hz) {
    apply_move_forward_for_one_sec_at_100hz();
    ASSERT_EQ(body_state_->time(), 1.0);
    ASSERT_TRUE(body_state_->getOrientationInGlobalFrame().isApprox(Quaternion::identity())) << "Orientation should be identity";
    Eigen::Vector3d expected_position;
    expected_position << 0.5, 0.0, 0.0;
    auto position = body_state_->getPositionInGlobalFrame();
    ASSERT_TRUE(position.isApprox(expected_position)) << "Got position [" << position.transpose() << "]^T";
    Eigen::Vector3d expected_velocity;
    expected_velocity << 1.0, 0.0, 0.0;
    auto velocity = body_state_->getVelocityInGlobalFrame();
    ASSERT_TRUE(velocity.isApprox(expected_velocity)) << "Got velocity [" << velocity.transpose() << "]^T";
}

TEST_F(BodyStateNoMovementTest, test_rotate_clockwise_for_one_sec_at_100hz) {
    apply_rotate_clockwise_for_one_sec_at_100hz();
    ASSERT_EQ(body_state_->time(), 1.0);
    Quaternion expected_orientation(0.0, 0.0, std::sin(M_PI/4.0), -std::cos(M_PI/4.0));
    
    Quaternion orientation = body_state_->getOrientationInGlobalFrame();
    ASSERT_TRUE(orientation.isApprox(expected_orientation, 1e-9)) << "Got orientation " << orientation.coeffs();
    
    auto position = body_state_->getPositionInGlobalFrame();
    ASSERT_TRUE(position.isZero(1e-12)) << "Got position [" << position.transpose() << "]^T";
    
    auto velocity = body_state_->getVelocityInGlobalFrame();
    ASSERT_TRUE(velocity.isZero(1e-12)) << "Got velocity [" << velocity.transpose() << "]^T";
}

TEST_F(BodyStateNoMovementTest, test_rotate_and_go) {
    apply_rotate_and_go();
    ASSERT_EQ(body_state_->time(), 2.0);
    Quaternion expected_orientation(0.0, 0.0, std::sin(M_PI/4.0), -std::cos(M_PI/4.0));
    
    Quaternion orientation = body_state_->getOrientationInGlobalFrame();
    ASSERT_TRUE(orientation.isApprox(expected_orientation, 1e-3)) << "Got orientation " << orientation.coeffs();
    
    Eigen::Vector3d expected_position;
    expected_position << 0.0, -0.5, 0.0;
    auto position = body_state_->getPositionInGlobalFrame();
    ASSERT_TRUE(position.isApprox(expected_position, 1e-8)) << "Got position [" << position.transpose() << "]^T";
    
    Eigen::Vector3d expected_velocity;
    expected_velocity << 0.0, -1.0, 0.0;
    auto velocity = body_state_->getVelocityInGlobalFrame();
    ASSERT_TRUE(velocity.isApprox(expected_velocity, 1e-8)) << "Got velocity [" << velocity.transpose() << "]^T";
}

class BodyStatePathTrajectoryTest : public ::testing::Test {
public:
    std::shared_ptr<MockCalibration> calibration_;
    std::shared_ptr<BodyState> body_state_;
    
    BodyStatePathTrajectoryTest() {
        calibration_ = std::shared_ptr<MockCalibration>(new MockCalibration);
        Eigen::Vector3d global_gravity;
        global_gravity << 0.0, 0.0, -9.81;
        calibration_->global_gravity_ = global_gravity;
    }
};

TEST_F(BodyStatePathTrajectoryTest, test_body_state) {
    Eigen::Vector3d rotation_estimate_1;
    rotation_estimate_1 << 0.0, 0.0, M_PI;
    Eigen::Vector3d rotation_estimate_2;
    rotation_estimate_2 << 0.0, 0.0, -M_PI;
    Eigen::Vector3d acceleration_estimate_1;
    acceleration_estimate_1 << 0.0, M_PI*M_PI, 9.81;
    Eigen::Vector3d acceleration_estimate_2;
    acceleration_estimate_2 << 0.0, -1*M_PI*M_PI, 9.81;
    
    Quaternion q_B_G = Quaternion::identity();
    Eigen::Vector3d p_B_G = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_B_G = M_PI * Eigen::Vector3d::UnitX();
    body_state_ = std::make_shared<BodyState>(calibration_, 0.0, rotation_estimate_1, acceleration_estimate_1, q_B_G, p_B_G, v_B_G);
    
    Eigen::Vector3d position;
    Quaternion orientation = Quaternion::identity();
    
    for (int i = 1; i < 10001; ++i) {
        body_state_ = BodyState::propagate(*body_state_, i/10000.0, rotation_estimate_1, acceleration_estimate_1);
    }
    
    position << 0.0, 2.0, 0.0;
    ASSERT_TRUE((body_state_->getPositionInGlobalFrame() - position).isZero(1e-3)) << "Got position [" << body_state_->getPositionInGlobalFrame().transpose() << "]^T";
    orientation = Quaternion(0, 0, 1, 0);
    ASSERT_TRUE(body_state_->getOrientationInGlobalFrame().isApprox(orientation, 1e-5)) << "Got orientation [" << body_state_->getOrientationInGlobalFrame().coeffs().transpose() << "]^T";
    
    for (int i = 1; i < 10001; ++i) {
        body_state_ = BodyState::propagate(*body_state_, 1 + i/10000.0, rotation_estimate_2, acceleration_estimate_2);
    }
    
    position << 0.0, 4.0, 0.0;
    ASSERT_TRUE((body_state_->getPositionInGlobalFrame() - position).isZero(1e-3)) << "Got position [" << body_state_->getPositionInGlobalFrame().transpose() << "]^T";
    orientation = Quaternion::identity();
    ASSERT_TRUE(body_state_->getOrientationInGlobalFrame().isApprox(orientation, 1e-3)) << "Got orientation [" << body_state_->getOrientationInGlobalFrame().coeffs().transpose() << "]^T";
}

