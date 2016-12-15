#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "feature_track.h"
#include "filter.h"
#include "filter_state.h"
#include "body_state.h"
#include "calibration.h"
#include "state_initializer.h"

class BigCameraTest;

class MockCalibration : public Calibration {
public:
    friend class BigCameraTest;
};

class MockBodyState : public BodyState {
public:
    friend class BigCameraTest;
    MockBodyState(std::shared_ptr<const Calibration> calibration, double time, Eigen::Vector3d rotation_estimate, Eigen::Vector3d acceleration_estimate, Eigen::Quaterniond q_B_G, Eigen::Vector3d p_B_G, Eigen::Vector3d v_B_G) : BodyState(calibration, time, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G) { }
};

class MockFilter : public Filter {
public:
    friend class BigCameraTest;
    MockFilter(std::shared_ptr<const Calibration> calibration) : Filter(calibration, std::make_shared<StateInitializer>()) { }
};

class MockFilterState : public FilterState {
public:
    friend class BigCameraTest;
    MockFilterState(std::shared_ptr<const Calibration> calibration) : FilterState(calibration) { }
};

class BigCameraTest : public ::testing::Test {
public:
    std::shared_ptr<MockCalibration> calibration_;
    std::shared_ptr<MockBodyState> body_state_;
    std::shared_ptr<MockFilter> filter_;
    std::shared_ptr<MockFilterState> filter_state_;
    std::shared_ptr<ImuBuffer> accel_buffer_;
    std::shared_ptr<ImuBuffer> gyro_buffer_;

    BigCameraTest() {
        std::cout << "Create BigCameraTest::BigCameraTest()" << std::endl;
        calibration_ = std::make_shared<MockCalibration>();
        Eigen::Vector3d global_gravity;
        global_gravity << 0.0, 0.0, -9.81;
        calibration_->global_gravity_ = global_gravity;
        calibration_->body_to_camera_rotation_ = Eigen::Quaterniond(-0.5, -0.5, 0.5, -0.5);
        std::cout << calibration_->body_to_camera_rotation_.coeffs() << std::endl;
        calibration_->max_camera_poses_ = 4;

        std::cout << "Už to bude rok" << std::endl;
        std::cout << calibration_->body_to_camera_rotation_.toRotationMatrix() << std::endl;
        std::cout << "co lízal jsem ti pekáč" << std::endl;

        filter_ = std::make_shared<MockFilter>(calibration_);
        filter_state_ = std::make_shared<MockFilterState>(calibration_);
        filter_->filter_state_ = filter_state_;
        filter_->filter_covar_ = Eigen::MatrixXd::Zero(56, 56);
        filter_->frame_rows_ = 1.0;

        Eigen::Vector2d optical_center;
        optical_center << 320, 240;
        filter_state_->optical_center_ = optical_center;
        Eigen::Vector2d focal_point;
        focal_point << 600, 600;
        filter_state_->focal_point_ = focal_point;
        Eigen::Vector3d position_of_body_in_camera;
        position_of_body_in_camera << 0.0, -0.1, 1.0;
        filter_state_->position_of_body_in_camera_ = position_of_body_in_camera;
        filter_state_->camera_delay_ = 0.0;
        filter_state_->camera_readout_ = 0.0;
        filter_state_->radial_distortion_ = Eigen::Vector3d::Zero();
        filter_state_->tangential_distortion_ = Eigen::Vector2d::Zero();
        filter_state_->gyroscope_shape_ = Eigen::Matrix3d::Identity();
        filter_state_->gyroscope_acceleration_sensitivity_ = Eigen::Matrix3d::Zero();
        filter_state_->accelerometer_shape_ = Eigen::Matrix3d::Identity();
        filter_state_->bias_gyroscope_ = Eigen::Vector3d::Zero();
        filter_state_->bias_accelerometer_ = Eigen::Vector3d::Zero();
    }

    void do_big_test() {
        accel_buffer_ = std::make_shared<ImuBuffer>();
        accel_buffer_->push_back(ImuItem::fromVector3d(-1.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        accel_buffer_->push_back(ImuItem::fromVector3d(0.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        accel_buffer_->push_back(ImuItem::fromVector3d(1.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        gyro_buffer_ = std::make_shared<ImuBuffer>();
        gyro_buffer_->push_back(ImuItem::fromVector3d(-1.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
        gyro_buffer_->push_back(ImuItem::fromVector3d(0.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
        gyro_buffer_->push_back(ImuItem::fromVector3d(1.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));

        Eigen::Quaterniond delta_q(std::cos(-1*M_PI/4.0), 0.0, 0.0, std::sin(-1*M_PI/4.0));
        ASSERT_NEAR(delta_q.norm(), 1.0, 1e-8);

        Eigen::Quaterniond q_B0_G = Eigen::Quaterniond::Identity();
        Eigen::Vector3d p_B0_G;
        p_B0_G << -1.0, 0.0, 0.5;

        Eigen::Quaterniond q_B1_G = delta_q;
        Eigen::Vector3d p_B1_G;
        p_B1_G << 0.0, -1.0, 0.5;

        Eigen::Quaterniond q_B2_G = delta_q*delta_q;
        Eigen::Vector3d p_B2_G;
        p_B2_G << 1.0, 0.0, 0.5;

        Eigen::Quaterniond q_B3_G = delta_q*delta_q*delta_q;
        Eigen::Vector3d p_B3_G;
        //p_B3_G << 0.0, 1.0, 0.5;
        p_B3_G << 100.0, -54022.0, 657570.5;

        // B_0
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), q_B0_G, p_B0_G, Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));

        // B_1
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), q_B1_G, p_B1_G, Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));

        // B_2
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), q_B2_G, p_B2_G, Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));

        // B_3
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), q_B3_G, p_B3_G, Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
        
        ASSERT_EQ(filter_state_.get(), std::addressof(filter_->state()));
    }
    
    std::size_t getNumberOfCameraPoses() {
        return filter_->filter_state_->poses().size();
    }
    
    std::pair<bool, Eigen::Vector3d> triangulateGlobalFeaturePosition(const FeatureTrack &feature_track) {
        return filter_->triangulateGlobalFeaturePosition(feature_track);
    }

    FeatureRezidualizationResult rezidualizeFeature(const FeatureTrack& feature_track) {
        return filter_->rezidualizeFeature(feature_track);
    }
};

TEST_F(BigCameraTest, BigTest) {
    do_big_test();
    
    ASSERT_EQ(filter_state_->poses().size(), 4);
    
    Eigen::Vector3d p_B0_G;
    p_B0_G << -1.0, 0.0, 0.5;
    ASSERT_TRUE((p_B0_G - filter_state_->poses()[0].getBodyPositionInGlobalFrame()).isZero(1e-10));
    
    Eigen::Vector3d p_C0_G;
    p_C0_G << -2.0, 0.0, 0.4;
    ASSERT_TRUE((p_C0_G - filter_state_->poses()[0].getCameraPositionInGlobalFrame(*filter_)).isZero(1e-10));
    
    Eigen::Vector3d p_C1_G;
    p_C1_G << 0.0, -2.0, 0.4;
    std::cout << "Vole" << std::endl;
    std::cout << filter_state_->poses()[1].getBodyPositionInGlobalFrame() << std::endl;
    std::cout << filter_state_->poses()[1].getCameraPositionInGlobalFrame(*filter_) << std::endl;
    ASSERT_TRUE((p_C1_G - filter_state_->poses()[1].getCameraPositionInGlobalFrame(*filter_)).isZero(1e-10));
    
    Eigen::Vector3d p_C2_G;
    p_C2_G << 2.0, 0.0, 0.4;
    ASSERT_TRUE((p_C2_G - filter_state_->poses()[2].getCameraPositionInGlobalFrame(*filter_)).isZero(1e-10));
    
    Eigen::Vector3d p_C3_G;
    p_C3_G << 0.0, 2.0, 0.4;
    //ASSERT_TRUE((p_C3_G - filter_state_->poses()[3].getCameraPositionInGlobalFrame(*filter_)).isZero(1e-10));
    
    FeatureTrack feature_track;
    feature_track.addFeaturePosition(320, 300);
    feature_track.addFeaturePosition(320, 300);
    feature_track.addFeaturePosition(320, 300);
    
    std::cout << filter_state_->poses()[0].gyroHint()->getTime() << std::endl;
    std::cout << filter_state_->poses()[1].gyroHint()->getTime() << std::endl;
    std::cout << filter_state_->poses()[2].gyroHint()->getTime() << std::endl;
//    std::cout << filter_state_->poses()[3].gyroHint()->getTime() << std::endl;
//    std::cout << filter_state_->poses()[0].accelHint()->getTime() << std::endl;
//    std::cout << filter_state_->poses()[1].accelHint()->getTime() << std::endl;
//    std::cout << filter_state_->poses()[2].accelHint()->getTime() << std::endl;
//    std::cout << filter_state_->poses()[3].accelHint()->getTime() << std::endl;
    
    FeatureRezidualizationResult result = rezidualizeFeature(feature_track);
    
    
}
