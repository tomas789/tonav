//
// Created by Tomas Krejci on 10/21/17.
//

#include "odometry/tonav_calibration.h"
#include <json.hpp>
#include <fstream>
#include "custom_serialization.h"

using json = nlohmann::json;

std::shared_ptr<TonavCalibration> TonavCalibration::prepare(SimSetup *sim_setup, const std::string& fname) {
    std::shared_ptr<TonavCalibration> calibration(new TonavCalibration(sim_setup));
    
    const Trajectory& trajectory = sim_setup->getTrajectory();
    const Vision& vision = sim_setup->getVision();
    
    calibration->focal_length_ = vision.getFocalLength();
    calibration->optical_center_ = vision.getOpticalCenter();
    calibration->radial_distortion_ = vision.getRadialDistortion();
    calibration->tangential_distortion_ = vision.getTangentialDistortion();
    calibration->camera_delay_time_ = 0;
    calibration->camera_readout_time_ = 0;
    
    calibration->gyroscope_acceleration_sensitivity_matrix_ = Eigen::Matrix3d::Zero();
    calibration->gyroscope_shape_matrix_ = Eigen::Matrix3d::Identity();
    calibration->accelerometer_shape_matrix_ = Eigen::Matrix3d::Identity();
    calibration->gyroscope_bias_ = Eigen::Vector3d::Zero();
    calibration->accelerometer_bias_ = Eigen::Vector3d::Zero();
    calibration->global_gravity_ = trajectory.getGlobalGravity();
    
    std::ifstream stream(fname);
    json j;
    stream >> j;
    
    calibration->accelerometer_variance_ = j["accelerometer_variance"];
    calibration->gyroscope_variance_ = j["gyroscope_variance"];
    calibration->accelerometer_random_walk_variance_ = j["accelerometer_random_walk_variance"];
    calibration->gyroscope_random_walk_variance_ = j["gyroscope_random_walk_variance"];
    
    calibration->max_camera_poses_ = 30;
    calibration->max_triangulation_iterations_ = 100;
    
    calibration->orientation_noise_ = j["orientation_noise"];
    calibration->position_noise_ = j["position_noise"];
    calibration->velocity_noise_ = j["velocity_noise"];
    calibration->gyroscope_bias_noise_ = j["gyroscope_bias_noise"];
    calibration->accelerometer_bias_noise_ = j["accelerometer_bias_noise"];
    calibration->gyroscope_acceleration_sensitivity_matrix_noise_ = j["gyroscope_acceleration_sensitivity_noise"];
    calibration->gyroscope_shape_matrix_noise_ = j["gyroscope_shape_matrix_noise"];
    calibration->accelerometer_shape_matrix_noise_ = j["accelerometer_shape_matrix_noise"];
    calibration->position_of_body_in_camera_frame_noise_ = j["position_of_body_in_camera_frame_noise"];
    calibration->focal_length_noise_ = j["focal_length_noise"];
    calibration->optical_center_noise_ = j["optical_center_noise"];
    calibration->radial_distortion_noise_ = j["radial_distortion_noise"];
    calibration->tangential_distortion_noise_ = j["tangential_distortion_noise"];
    calibration->camera_delay_time_noise_ = j["camera_delay_time_noise"];
    calibration->camera_readout_time_noise_ = j["camera_readout_time_noise"];
    calibration->image_noise_variance_ = j["image_noise_variance"];
    
    calibration->body_to_camera_rotation_ = trajectory.getBodyToCameraFrameRotation();
    
    return calibration;
}

Eigen::Vector3d TonavCalibration::getPositionOfBodyInCameraFrame() const {
    const Trajectory &trajectory = sim_setup_->getTrajectory();
    Eigen::Vector3d p_C_B = trajectory.getCameraPositionInBodyFrame();
    tonav::Quaternion q_C_B = trajectory.getBodyToCameraFrameRotation();
    
    Eigen::Vector3d p_B_C = tonav::Geometry::switchFrames(p_C_B, q_C_B);
    return p_B_C;
}

TonavCalibration::TonavCalibration(SimSetup *sim_setup)
    : sim_setup_(sim_setup) {
    
}

