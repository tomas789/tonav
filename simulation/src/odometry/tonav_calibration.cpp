//
// Created by Tomas Krejci on 10/21/17.
//

#include "odometry/tonav_calibration.h"

std::shared_ptr<TonavCalibration> TonavCalibration::prepare(SimSetup *sim_setup) {
    std::shared_ptr<TonavCalibration> calibration(new TonavCalibration(sim_setup));
    
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

