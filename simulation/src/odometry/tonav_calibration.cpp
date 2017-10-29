//
// Created by Tomas Krejci on 10/21/17.
//

#include "odometry/tonav_calibration.h"

std::shared_ptr<TonavCalibration> TonavCalibration::prepare(SimSetup *sim_setup) {
    std::shared_ptr<TonavCalibration> calibration(new TonavCalibration(sim_setup));
    
    const Trajectory& trajectory = sim_setup->getTrajectory();
    const Vision& vision = sim_setup->getVision();
    
    calibration->focal_point_ = vision.getFocalLength();
    calibration->optical_center_ = vision.getOpticalCenter();
    calibration->radial_distortion_ = vision.getRadialDistortion();
    calibration->tangential_distortion_ = vision.getTangentialDistortion();
    calibration->camera_delay_time_ = 0;
    calibration->camera_readout_time_ = 0;
    calibration->image_noise_variance_ = 1e-9;
    
    calibration->gyroscope_acceleration_sensitivity_matrix_ = Eigen::Matrix3d::Zero();
    calibration->gyroscope_shape_matrix_ = Eigen::Matrix3d::Identity();
    calibration->accelerometer_shape_matrix_ = Eigen::Matrix3d::Identity();
    calibration->gyroscope_bias_ = Eigen::Vector3d::Zero();
    calibration->accelerometer_bias_ = Eigen::Vector3d::Zero();
    calibration->global_gravity_ = trajectory.getGlobalGravity();
    
    calibration->accelerometer_variance_ = 1e-9;
    calibration->gyroscope_variance_ = 1e-9;
    calibration->accelerometer_random_walk_variance_ = 1e-9;
    calibration->gyroscope_random_walk_variance_ = 1e-9;
    
    calibration->max_camera_poses_ = 30;
    calibration->max_triangulation_iterations_ = 100;
    
    Eigen::Vector3d ones3;
    ones3 << 1, 1, 1;
    
    Eigen::Vector2d ones2;
    ones2 << 1, 1;
    
    Eigen::Matrix3d ones33;
    ones33 << 1, 1, 1, 1, 1, 1, 1, 1, 1;
    
    calibration->orientation_noise_ = 1e-9*ones3;
    calibration->position_noise_ = 1e-9*ones3;
    calibration->velocity_noise_ = 1e-9*ones3;
    calibration->gyroscope_bias_noise_ = 1e-9*ones3;
    calibration->accelerometer_bias_noise_ = 1e-9*ones3;
    calibration->gyroscope_acceleration_sensitivity_matrix_noise_ = 1e-9*ones33;
    calibration->gyroscope_shape_matrix_noise_ = 1e-9*ones33;
    calibration->accelerometer_shape_matrix_noise_ = 1e-9*ones33;
    calibration->position_of_body_in_camera_frame_noise_ = 1e-9*ones3;
    calibration->focal_point_noise_ = 1e-9*ones2;
    calibration->optical_center_noise_ = 1e-9*ones2;
    calibration->radial_distortion_noise_ = 1e-9*ones3;
    calibration->tangential_distortion_noise_ = 1e-9*ones2;
    calibration->camera_delay_time_noise_ = 1e-9;
    calibration->camera_readout_time_noise_ = 1e-9;
    
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

