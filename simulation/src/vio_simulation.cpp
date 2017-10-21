//
// Created by Tomas Krejci on 10/7/17.
//

#include "vio_simulation.h"

#include <stdexcept>
#include <thread>
#include <mutex>
#include <opencv2/core/eigen.hpp>

VioSimulation::VioSimulation() = default;

void VioSimulation::run(std::shared_ptr<SimSetup> sim_setup) {
    if (!sim_setup) {
        throw std::runtime_error("No SimSetup provided.");
    }
    sim_setup_ = sim_setup;
    run_loop_.registerSimulationForUpdates(this);
    if (!window_) {
        window_.reset(new cv::viz::Viz3d("Vio Simulation"));
        window_->showWidget("Coordinate Frame", cv::viz::WCoordinateSystem());
        cv::Affine3d coord_frame_pose = getPose(tonav::Quaternion::identity(), Eigen::Vector3d::Zero());
        window_->setWidgetPose("Coordinate Frame", coord_frame_pose);
        camera_.reset(new cv::viz::WCameraPosition(sim_setup->getVision().getCameraCalibrationMatrix()));
        window_->showWidget("Camera", *camera_);
        window_->spinOnce();
    }
    
    sim_setup->getImu().initialize(this);
    sim_setup->getVision().initialize(this);
    sim_setup->getTrajectory().initialize(this);
    
    is_simulation_running_ = true;
    std::thread t(&VioSimulation::startRunLoop, this);
    
    while (is_simulation_running_) {
        std::lock_guard<std::mutex> _(ui_lock_);
        window_->spinOnce();
    }
    t.join();
}

void VioSimulation::accelerometerCallback(float time, Eigen::Vector3d accel) {
    std::cout << "Accelerometer: [" << accel.transpose() << "]^T" << std::endl;
}

void VioSimulation::gyroscopeCallback(float time, Eigen::Vector3d gyro) {
    std::cout << "Gyroscope: [" << gyro.transpose() << "]^T" << std::endl;
}

void VioSimulation::cameraCallback(float time, cv::Mat frame) {

}

void VioSimulation::runLoopCallback(float time) {
    std::cout << ((bool)sim_setup_) << std::endl;
    Trajectory& trajectory = sim_setup_->getTrajectory();
    Eigen::Vector3d p_B_G = trajectory.getCameraPositionInGlobalFrame(time);
    tonav::Quaternion q_G_B = trajectory.getGlobalToCameraFrameRotation(time).conjugate();
    
    {
        std::lock_guard<std::mutex> _(ui_lock_);
        window_->setWidgetPose("Camera", getPose(q_G_B, p_B_G));
        window_->getCamera().setClip(cv::Vec2d(0.5, 20));
    }
}

RunLoop& VioSimulation::getRunLoop() {
    return run_loop_;
}

const RunLoop& VioSimulation::getRunLoop() const {
    return run_loop_;
}

void VioSimulation::startRunLoop() {
    run_loop_.run();
    is_simulation_running_ = false;
}

cv::Affine3d VioSimulation::getPose(tonav::Quaternion q, Eigen::Vector3d p) const {
    cv::Matx33d R;
    cv::eigen2cv((q_viz_sim_*q).toRotationMatrix(), R);
    cv::Vec3d p_viz;
    Eigen::Vector3d p_viz_eigen = q_viz_sim_.toRotationMatrix()*p;
    cv::eigen2cv(p_viz_eigen, p_viz);
    cv::Affine3d pose(R, p_viz);
    return pose;
}
