//
// Created by Tomas Krejci on 10/8/17.
//

#include "trajectory/circular_trajectory.h"

#include <cmath>

std::unique_ptr<Trajectory> CircularTrajectory::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<CircularTrajectory> trajectory(new CircularTrajectory(sim_setup));
    
    json radius_json = j.at("radius");
    if (!radius_json.is_number())
        throw "'radius' has to be a number.";
    trajectory->radius_ = radius_json;
    
    json time_per_revolution_json = j.at("time_per_revolution");
    if (!time_per_revolution_json.is_number())
        throw "'time_per_revolution' has to be a number.";
    trajectory->time_per_revolution_ = time_per_revolution_json;
    
    json body_to_camera_rotation_json = j.at("body_to_camera_rotation");
    
    json body_to_camera_rotation_x_json = body_to_camera_rotation_json.at("x");
    if (!body_to_camera_rotation_x_json.is_number())
        throw "'body_to_camera_rotation.x' has to be a number.";
    double body_to_camera_rotation_x = body_to_camera_rotation_x_json;
    
    json body_to_camera_rotation_y_json = body_to_camera_rotation_json.at("y");
    if (!body_to_camera_rotation_y_json.is_number())
        throw "'body_to_camera_rotation.y' has to be a number.";
    double body_to_camera_rotation_y = body_to_camera_rotation_y_json;
    
    json body_to_camera_rotation_z_json = body_to_camera_rotation_json.at("z");
    if (!body_to_camera_rotation_z_json.is_number())
        throw "'body_to_camera_rotation.z' has to be a number.";
    double body_to_camera_rotation_z = body_to_camera_rotation_z_json;
    
    json body_to_camera_rotation_w_json = body_to_camera_rotation_json.at("w");
    if (!body_to_camera_rotation_w_json.is_number())
        throw "'body_to_camera_rotation.w' has to be a number.";
    double body_to_camera_rotation_w = body_to_camera_rotation_w_json;
    
    trajectory->q_C_B_ = tonav::Quaternion(
        body_to_camera_rotation_x,
        body_to_camera_rotation_y,
        body_to_camera_rotation_z,
        body_to_camera_rotation_w
    );
    if (std::abs(trajectory->q_C_B_.norm() - 1) >= 1e-6) {
        throw std::runtime_error("'body_to_camera_rotation' has to be a unit quaternion (eps 1e-6).");
    }
    
    json camera_in_body_position_json = j.at("camera_in_body_position");
    std::vector<double> camera_in_body_position = camera_in_body_position_json;
    if (camera_in_body_position.size() != 3)
        throw std::runtime_error("'camera_in_body_position' has to be an array of length 3.");
    trajectory->p_C_B_ << camera_in_body_position[0], camera_in_body_position[1], camera_in_body_position[2];
    
    return std::move(trajectory);
}

void CircularTrajectory::initialize(VioSimulation *simulation) {

}

Eigen::Vector3d CircularTrajectory::getBodyPositionInGlobalFrame(double time) {
    double t = M_2_PI*time/time_per_revolution_;
    double s = std::sin(t);
    double c = std::cos(t);
    Eigen::Vector3d position;
    position << radius_*s, radius_*c, 0;
    return position;
}

tonav::Quaternion CircularTrajectory::getGlobalToBodyFrameRotation(double time) {
    double theta = M_2_PI*time/time_per_revolution_;
    return tonav::Quaternion(0, 0, std::sin(theta/2), std::cos(theta/2)).conjugate();
}

Eigen::Vector3d CircularTrajectory::getCameraPositionInGlobalFrame(double time) {
    Eigen::Vector3d p_B_G = getBodyPositionInGlobalFrame(time);
    Eigen::Matrix3d R_B_G = getGlobalToBodyFrameRotation(time).toRotationMatrix();
    Eigen::Matrix3d R_G_B = R_B_G.transpose();
    Eigen::Vector3d p_G_B = -R_B_G*p_B_G;
    Eigen::Vector3d p_C_G = R_G_B*(p_C_B_ - p_G_B);
    return p_C_G;
}

tonav::Quaternion CircularTrajectory::getGlobalToCameraFrameRotation(double time) {
    tonav::Quaternion q_B_G = getGlobalToBodyFrameRotation(time);
    return q_C_B_*q_B_G;
}

CircularTrajectory::~CircularTrajectory() = default;

CircularTrajectory::CircularTrajectory(SimSetup *sim_setup)
    : Trajectory(sim_setup),
      q_C_B_(tonav::Quaternion::identity()) {
    
}

