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

class CameraProjectionTest;

class MockCalibration : public Calibration {
public:
    friend class CameraProjectionTest;
};

class MockBodyState : public BodyState {
public:
    friend class CameraProjectionTest;
    MockBodyState(std::shared_ptr<const Calibration> calibration, double time, Eigen::Vector3d rotation_estimate, Eigen::Vector3d acceleration_estimate, Eigen::Quaterniond q_B_G, Eigen::Vector3d p_B_G, Eigen::Vector3d v_B_G) : BodyState(calibration, time, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G) { }
};

class MockFilter : public Filter {
public:
    friend class CameraProjectionTest;
    MockFilter(std::shared_ptr<const Calibration> calibration) : Filter(calibration, std::make_shared<StateInitializer>()) { }
};

class MockFilterState : public FilterState {
public:
    friend class CameraProjectionTest;
    MockFilterState(std::shared_ptr<const Calibration> calibration) : FilterState(calibration) { }
};

class CameraProjectionTest : public ::testing::Test {
public:
    std::shared_ptr<MockCalibration> calibration_;
    std::shared_ptr<MockBodyState> body_state_;
    std::shared_ptr<MockFilter> filter_;
    std::shared_ptr<MockFilterState> filter_state_;
    std::shared_ptr<ImuBuffer> accel_buffer_;
    std::shared_ptr<ImuBuffer> gyro_buffer_;

    
    CameraProjectionTest() {
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
        Eigen::Vector3d camera_position_in_body_frame;
        camera_position_in_body_frame << 0, 0, 0;
        filter_state_->position_of_body_in_camera_ = camera_position_in_body_frame;
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
    
    void apply_move_backwards() {
        accel_buffer_ = std::make_shared<ImuBuffer>();
        accel_buffer_->push_back(ImuItem::fromVector3d(-1.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        accel_buffer_->push_back(ImuItem::fromVector3d(0.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        accel_buffer_->push_back(ImuItem::fromVector3d(1.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        gyro_buffer_ = std::make_shared<ImuBuffer>();
        gyro_buffer_->push_back(ImuItem::fromVector3d(-1.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
        gyro_buffer_->push_back(ImuItem::fromVector3d(0.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
        gyro_buffer_->push_back(ImuItem::fromVector3d(1.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
        
        Eigen::Matrix3d R_C_B = calibration_->body_to_camera_rotation_.toRotationMatrix();
        Eigen::Vector3d p_x_C;
        p_x_C << 1, 2, 3;
        Eigen::Vector3d p_x_B;
        p_x_B << 3, -1, -2;
        ASSERT_TRUE((R_C_B*p_x_B - p_x_C).isZero(1e-12));
        
        // p_B_G: (-1, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), -1*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
        
        
        // p_B_G: (-2, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), -2*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
        
        // p_B_G: (-3, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), -3*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
        
        // p_B_G:(-4, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), -4*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
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

TEST_F(CameraProjectionTest, test_runs_ok) {
    apply_move_backwards( );
    
    ASSERT_EQ(getNumberOfCameraPoses(), 4);
    
    ASSERT_TRUE(Eigen::Quaterniond::Identity().angularDistance(filter_state_->poses()[0].getBodyOrientationInGlobalFrame()) < 1e-12);
    ASSERT_TRUE(Eigen::Quaterniond::Identity().angularDistance(filter_state_->poses()[1].getBodyOrientationInGlobalFrame()) < 1e-12);
    ASSERT_TRUE(Eigen::Quaterniond::Identity().angularDistance(filter_state_->poses()[2].getBodyOrientationInGlobalFrame()) < 1e-12);
    ASSERT_TRUE(Eigen::Quaterniond::Identity().angularDistance(filter_state_->poses()[3].getBodyOrientationInGlobalFrame()) < 1e-12);
    
    FeatureTrack feature_track;
    feature_track.addFeaturePosition(320, 90);
    feature_track.addFeaturePosition(320, 140);
    feature_track.addFeaturePosition(320, 165);
    
    std::pair<bool, Eigen::Vector3d> position = triangulateGlobalFeaturePosition(feature_track);
    
    Eigen::Vector3d p_f_G;
    p_f_G << 1.0, 0.0, 0.5;
    
    ASSERT_TRUE((position.second - p_f_G).isZero(1e-6)) << "Got global position [" << position.second.transpose() << "]^T";
    
//    FeatureRezidualizationResult result = rezidualizeFeature(feature_track);
}
