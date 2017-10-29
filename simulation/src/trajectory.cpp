//
// Created by Tomas Krejci on 10/8/17.
//

#include "trajectory.h"

#include <limits>

#include "custom_serialization.h"
#include "trajectory/circular_trajectory.h"
#include "trajectory/inplace_rotation_trajectory.h"
#include "trajectory/standstill_trajectory.h"


std::unique_ptr<Trajectory> Trajectory::load(SimSetup *sim_setup, const json &j) {
    std::string type = j.at("type").get<std::string>();
    json params = j.at("params");
    
    std::unique_ptr<Trajectory> trajectory;
    
    if (type == "circular") {
        trajectory = std::move(CircularTrajectory::load(sim_setup, params));
    } else if (type == "standstill") {
        trajectory = std::move(StandstillTrajectory::load(sim_setup, params));
    } else if (type == "inplace_rotation") {
        trajectory = std::move(InplaceRotationTrajectory::load(sim_setup, params));
    } else {
        throw std::runtime_error("Unknown trajectory type.");
    }
    
    trajectory->q_C_B_ = std::move(params.at("body_to_camera_rotation").get<tonav::Quaternion>());
    if (std::abs(trajectory->q_C_B_.norm() - 1) >= 1e-6) {
        throw std::runtime_error("'body_to_camera_rotation' has to be a unit quaternion (eps 1e-6).");
    }
    
    json camera_in_body_position_json = params.at("camera_in_body_position");
    std::vector<double> camera_in_body_position = camera_in_body_position_json;
    if (camera_in_body_position.size() != 3)
        throw std::runtime_error("'camera_in_body_position' has to be an array of length 3.");
    trajectory->p_C_B_ << camera_in_body_position[0], camera_in_body_position[1], camera_in_body_position[2];
    
    return std::move(trajectory);
}

Eigen::Vector3d Trajectory::getCameraPositionInGlobalFrame(double time) const {
    Eigen::Vector3d p_B_G = getBodyPositionInGlobalFrame(time);
    Eigen::Matrix3d R_B_G = getGlobalToBodyFrameRotation(time).toRotationMatrix();
    Eigen::Matrix3d R_G_B = R_B_G.transpose();
    Eigen::Vector3d p_G_B = -R_B_G*p_B_G;
    Eigen::Vector3d p_C_G = R_G_B*(p_C_B_ - p_G_B);
    return p_C_G;
}

tonav::Quaternion Trajectory::getGlobalToCameraFrameRotation(double time) const {
    tonav::Quaternion q_B_G = getGlobalToBodyFrameRotation(time);
    return q_C_B_*q_B_G;
}

Eigen::Vector3d Trajectory::getCameraPositionInBodyFrame() const {
    return p_C_B_;
}

tonav::Quaternion Trajectory::getBodyToCameraFrameRotation() const {
    return q_C_B_;
}

Trajectory::~Trajectory() = default;

Trajectory::Trajectory(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }