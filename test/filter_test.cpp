#include <gtest/gtest.h>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <valarray>
#include <fstream>
#include <sstream>

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
    friend class MatlabGlobalFeaturePositionTest;
};

class MockBodyState : public BodyState {
public:
    friend class CameraProjectionTest;
    friend class MatlabGlobalFeaturePositionTest;
    MockBodyState(std::shared_ptr<const Calibration> calibration, double time, Eigen::Vector3d rotation_estimate, Eigen::Vector3d acceleration_estimate, Quaternion q_B_G, Eigen::Vector3d p_B_G, Eigen::Vector3d v_B_G) : BodyState(calibration, time, rotation_estimate, acceleration_estimate, q_B_G, p_B_G, v_B_G) { }
};

class MockFilter : public Filter {
public:
    friend class CameraProjectionTest;
    friend class MatlabGlobalFeaturePositionTest;
    MockFilter(std::shared_ptr<const Calibration> calibration) : Filter(calibration, std::make_shared<StateInitializer>()) { }
};

class MockFilterState : public FilterState {
public:
    friend class CameraProjectionTest;
    friend class MatlabGlobalFeaturePositionTest;
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
        calibration_->body_to_camera_rotation_ = Quaternion(-0.5, 0.5, -0.5, -0.5);
        std::cout << calibration_->body_to_camera_rotation_.coeffs() << std::endl;
        calibration_->max_camera_poses_ = 4;
        
        std::cout << calibration_->body_to_camera_rotation_.toRotationMatrix() << std::endl;
        
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
        ASSERT_TRUE((R_C_B*p_x_B - p_x_C).isZero(1e-6));
        
        // p_B_G: (-1, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Quaternion::identity(), -1*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
        
        
        // p_B_G: (-2, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Quaternion::identity(), -2*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
        
        // p_B_G: (-3, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Quaternion::identity(), -3*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
        
        // p_B_G:(-4, 0, 0)^T
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Quaternion::identity(), -4*Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
    }
    
    std::size_t getNumberOfCameraPoses() {
        return filter_->filter_state_->poses().size();
    }
    
    std::pair<bool, Eigen::Vector3d> triangulateGlobalFeaturePosition(const FeatureTrack &feature_track) {
        return filter_->cameraAlgorithms().triangulateGlobalFeaturePosition(feature_track);
    }
    
    FeatureRezidualizationResult rezidualizeFeature(const FeatureTrack& feature_track) {
        cv::Mat mat;
        return filter_->rezidualizeFeature(feature_track, mat);
    }
    
    Eigen::Vector3d initialGuessFeaturePosition(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, const Eigen::Matrix3d& R_C0_C1, const Eigen::Vector3d& p_C1_C0, InitialGuessMethod method) const {
        return filter_->cameraAlgorithms().initialGuessFeaturePosition(z0, z1, R_C0_C1, p_C1_C0, method);
    }
};

TEST_F(CameraProjectionTest, test_runs_ok) {
    apply_move_backwards( );
    
    ASSERT_EQ(getNumberOfCameraPoses(), 4);
    
    ASSERT_TRUE(Quaternion::identity().isApprox(filter_state_->poses()[0].getBodyOrientationInGlobalFrame(), 1e-12));
    ASSERT_TRUE(Quaternion::identity().isApprox(filter_state_->poses()[1].getBodyOrientationInGlobalFrame(), 1e-12));
    ASSERT_TRUE(Quaternion::identity().isApprox(filter_state_->poses()[2].getBodyOrientationInGlobalFrame(), 1e-12));
    ASSERT_TRUE(Quaternion::identity().isApprox(filter_state_->poses()[3].getBodyOrientationInGlobalFrame(), 1e-12));
    
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

TEST_F(CameraProjectionTest, test_initialize_from_msckf_swf_comparison) {
    Eigen::Vector2d obs1;
    obs1 << 0.2315, -0.0226;
    Eigen::Vector2d obs2;
    obs2 << -0.1304, -0.0057;
    Eigen::Matrix3d C_12;
    C_12 << 0.9353, 0.0054, 0.3538, 0.0025, 0.9998, -0.0217, 0.0025, 0.9998, -0.0217;
    Eigen::Vector3d t_21_1;
    t_21_1 << 11.5050, -0.9159, 48.6971;
    Eigen::Vector3d p_f1_1;
    p_f1_1 << 11.1910, -1.0947, 48.3465;
    
    Eigen::Vector3d global_position = initialGuessFeaturePosition(obs1, obs2, C_12, t_21_1, InitialGuessMethod::SVD);
    // ASSERT_TRUE((global_position - p_f1_1).isZero(1e-6)) << "Got position [" << global_position.transpose() << "]^T";
}

class MatlabGlobalFeaturePositionTest : public ::testing::Test {
public:
    std::shared_ptr<MockCalibration> calibration_;
    std::shared_ptr<MockBodyState> body_state_;
    std::shared_ptr<MockFilter> filter_;
    std::shared_ptr<MockFilterState> filter_state_;
    std::shared_ptr<ImuBuffer> accel_buffer_;
    std::shared_ptr<ImuBuffer> gyro_buffer_;
    
    std::vector<std::valarray<double>> states_;
    std::map<int, std::map<int, std::vector<std::valarray<double>>>> tracks_;
    
    
    MatlabGlobalFeaturePositionTest() {
        calibration_ = std::make_shared<MockCalibration>();
        Eigen::Vector3d global_gravity;
        global_gravity << 0.0, 0.0, -9.81;
        calibration_->global_gravity_ = global_gravity;
        calibration_->body_to_camera_rotation_ = Quaternion(-0.4992, 0.4966, -0.5029, -0.5013);
        std::cout << calibration_->body_to_camera_rotation_.coeffs() << std::endl;
        calibration_->max_camera_poses_ = 400;
        
        std::cout << calibration_->body_to_camera_rotation_.toRotationMatrix() << std::endl;
        
        filter_ = std::make_shared<MockFilter>(calibration_);
        filter_state_ = std::make_shared<MockFilterState>(calibration_);
        filter_->filter_state_ = filter_state_;
        filter_->filter_covar_ = Eigen::MatrixXd::Zero(56, 56);
        filter_->frame_rows_ = 1.0;
        
        Eigen::Vector2d optical_center;
        optical_center << 609.5593, 172.854;
        filter_state_->optical_center_ = optical_center;
        Eigen::Vector2d focal_point;
        focal_point << 721.5377, 721.5377;
        filter_state_->focal_point_ = focal_point;
        Eigen::Vector3d camera_position_in_body_frame;
        camera_position_in_body_frame << 1.0833, -0.3099, 0.7299;
        filter_state_->position_of_body_in_camera_ = -1 * calibration_->body_to_camera_rotation_.toRotationMatrix() * camera_position_in_body_frame;
        filter_state_->camera_delay_ = 0.0;
        filter_state_->camera_readout_ = 0.0;
        filter_state_->radial_distortion_ = Eigen::Vector3d::Zero();
        filter_state_->tangential_distortion_ = Eigen::Vector2d::Zero();
        filter_state_->gyroscope_shape_ = Eigen::Matrix3d::Identity();
        filter_state_->gyroscope_acceleration_sensitivity_ = Eigen::Matrix3d::Zero();
        filter_state_->accelerometer_shape_ = Eigen::Matrix3d::Identity();
        filter_state_->bias_gyroscope_ = Eigen::Vector3d::Zero();
        filter_state_->bias_accelerometer_ = Eigen::Vector3d::Zero();
        
        accel_buffer_ = std::make_shared<ImuBuffer>();
        accel_buffer_->push_back(ImuItem::fromVector3d(-1.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        accel_buffer_->push_back(ImuItem::fromVector3d(0.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        accel_buffer_->push_back(ImuItem::fromVector3d(1.0, ImuDevice::ACCELEROMETER, Eigen::Vector3d::Zero()));
        gyro_buffer_ = std::make_shared<ImuBuffer>();
        gyro_buffer_->push_back(ImuItem::fromVector3d(-1.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
        gyro_buffer_->push_back(ImuItem::fromVector3d(0.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
        gyro_buffer_->push_back(ImuItem::fromVector3d(1.0, ImuDevice::GYROSCOPE, Eigen::Vector3d::Zero()));
    }
    
    void loadStates() {
        std::string file_name = "/Users/tomaskrejci/msckf_states.csv";
        std::ifstream file(file_name);
        if (!file) {
            throw std::runtime_error("File cannot be opened.");
        }
        for (std::string line; std::getline(file, line); ) {
            std::valarray<double> row(7);
            std::ostringstream row_stream(line);
            std::istringstream stream(line);
            for (std::size_t i = 0; i < 7; ++i) {
                std::string item;
                std::getline(stream, item, ',');
                row[i] = std::atof(item.c_str());
//                std::cout << i << ": " << row[i] << std::endl;
            }
            states_.push_back(row);
        }
    }
    
    void loadTracks() {
        std::string file_name = "/Users/tomaskrejci/msckf_tracks.csv";
        std::ifstream file(file_name);
        if (!file) {
            throw std::runtime_error("File cannot be opened.");
        }
        for (std::string line; std::getline(file, line); ) {
            std::valarray<double> row(8);
            std::ostringstream row_stream(line);
            std::istringstream stream(line);
            for (std::size_t i = 0; i < 8; ++i) {
                std::string item;
                std::getline(stream, item, ',');
                row[i] = std::atof(item.c_str());
//                std::cout << i << ": " << row[i] << std::endl;
            }
            tracks_[row[0]][row[1]-1].push_back(row);
        }
    }
    
    void augment(const Quaternion& q_B_G, const Eigen::Vector3d& p_B_G) {
        body_state_ = std::make_shared<MockBodyState>(calibration_, 0.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), q_B_G, p_B_G, Eigen::Vector3d::Zero());
        static_cast<MockFilterState&>(filter_->state()).body_state_ = body_state_;
        filter_->augment(std::next(std::begin(*gyro_buffer_)), std::next(std::begin(*accel_buffer_)));
    }
    
    std::pair<bool, Eigen::Vector3d> triangulateGlobalFeaturePosition(const FeatureTrack &feature_track) {
        return filter_->cameraAlgorithms().triangulateGlobalFeaturePosition(feature_track);
    }
    
    FeatureRezidualizationResult rezidualizeFeature(const FeatureTrack& feature_track) {
        cv::Mat mat;
        return filter_->rezidualizeFeature(feature_track, mat);
    }
};

TEST_F(MatlabGlobalFeaturePositionTest, test_load_states) {
    loadStates();
    loadTracks();
    
    for (std::size_t i = 0; i < 25; ++i) {
        Quaternion q_G_B(states_[i][0], states_[i][1], states_[i][2], states_[i][3]);
        Eigen::Vector3d p_B_G;
        p_B_G << states_[i][4], states_[i][5], states_[i][6];
        std::cout << "R_G_B(" << i << "): \n" << q_G_B.toRotationMatrix() << std::endl;
        std::cout << "p_B_G(" << i << "): \n" << p_B_G.transpose() << std::endl;
        // std::cout << "State " << i << ": q:" << q_G_B.conjugate().coeffs().transpose() << " p: " << p_B_G.transpose() << std::endl;
        augment(q_G_B.conjugate(), p_B_G);
        const CameraPoseBuffer& buffer = filter_state_->poses();
        std::cout << "q_C_G " << buffer[buffer.size()-1].getCameraOrientationInGlobalFrame(*filter_).coeffs().transpose() << std::endl;
        
        for (std::size_t k = 0; k <= i; ++k) {
            Quaternion q_G_Bk(states_[k][0], states_[k][1], states_[k][2], states_[k][3]);
            Eigen::Vector3d p_Bk_G;
            p_Bk_G << states_[k][4], states_[k][5], states_[k][6];
            
            ASSERT_TRUE((buffer[k].getBodyPositionInGlobalFrame() - p_Bk_G).isZero());
            ASSERT_TRUE(buffer[k].getBodyOrientationInGlobalFrame().isApprox(q_G_Bk.conjugate(), 1e-10));
        }
        
        std::cout << "Augment_" << i << "_(q_B_G: " << q_G_B.conjugate().coeffs().transpose() << ", p_B_G: " << p_B_G.transpose() << ")" << std::endl;
        std::cout << "R_B_G: \n" << q_G_B.conjugate().toRotationMatrix() << std::endl;
        
        auto it_tracks = tracks_.find(i);
        if (it_tracks != std::end(tracks_)) {
            for (const std::pair<int, std::vector<std::valarray<double>>>& track_pair : it_tracks->second) {
                const std::vector<std::valarray<double>>& track_data = track_pair.second;
                
                FeatureTrack track;
                for (std::size_t j = 0; j < track_data.size(); ++j) {
                    double x = track_data[j][3] * 721.5377 + 609.5593;
                    double y = track_data[j][4] * 721.5377 + 172.854;
                    // std::cout << "PX: " << x << ", " << y << std::endl;
                    track.addFeaturePosition(x, y);
                }
                
                Eigen::Vector3d p_f_G_matlab;
                p_f_G_matlab << track_data[0][5], track_data[0][6], track_data[0][7];
                
//                const CameraPose& C0 = filter_state_->poses()[filter_state_->poses().size()-1-track_data.size()];
//                const CameraPose& Clast = filter_state_->poses()[filter_state_->poses().size()-2];
//                Eigen::Matrix3d C_12 = Clast.getRotationToOtherPose(C0, *filter_).toRotationMatrix();
//                std::cout << "C_12: \n" << C_12 << std::endl;
//                std::cout << "C_12(Matlab version): \n" << (C0.getCameraOrientationInGlobalFrame(*filter_).toRotationMatrix()*Clast.getCameraOrientationInGlobalFrame(*filter_).toRotationMatrix().transpose()) << std::endl;
//                Eigen::Vector3d t_21_1 = C0.getPositionOfAnotherPose(Clast, *filter_);
//                std::cout << "t_21_1: \n" << t_21_1.transpose() << std::endl;
//                
//                std::cout << "R_B0_G: \n" << C0.getBodyOrientationInGlobalFrame().toRotationMatrix() << std::endl;
//                std::cout << "R_C0_G: \n" << C0.getCameraOrientationInGlobalFrame(*filter_).toRotationMatrix() << std::endl;
//                std::cout << "R_Blast_G: \n" << Clast.getBodyOrientationInGlobalFrame().toRotationMatrix() << std::endl;
//                std::cout << "R_Clast_G: \n" << Clast.getCameraOrientationInGlobalFrame(*filter_).toRotationMatrix() << std::endl;
//                
//                std::cout << "p_C0_G: " << C0.getCameraPositionInGlobalFrame(*filter_).transpose() << std::endl;
//                std::cout << "p_B0_G: " << C0.getBodyPositionInGlobalFrame().transpose() << std::endl;
//                std::cout << "p_Clast_G: " << Clast.getCameraPositionInGlobalFrame(*filter_).transpose() << std::endl;
//                std::cout << "p_Blast_G: " << Clast.getBodyPositionInGlobalFrame().transpose() << std::endl;
//                
//                std::cout << "R_C_B: " << calibration_->getBodyToCameraRotation().toRotationMatrix() << std::endl;
                
                std::pair<bool, Eigen::Vector3d> result = triangulateGlobalFeaturePosition(track);
                ASSERT_TRUE(result.first);
                Eigen::Vector3d p_f_G = result.second;
                std::cout << "p_f_G (state_k: " << track_data[0][0] << ", track: " << track_data[0][1] << "): " << ": [" << p_f_G.transpose() << "]^T (diff [" << (p_f_G - p_f_G_matlab).transpose() << "]^T)" << std::endl;
            }
        }
    }
}
