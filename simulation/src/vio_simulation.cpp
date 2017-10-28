//
// Created by Tomas Krejci on 10/7/17.
//

#include "vio_simulation.h"

#include <stdexcept>
#include <thread>
#include <mutex>
#include <opencv2/core/eigen.hpp>
#include <tonav.h>

VioSimulation* VioSimulation::global_vio_simulation_;

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
    
        camera_gt_.reset(new cv::viz::WCameraPosition(sim_setup->getVision().getCameraCalibrationMatrix(), 1, cv::viz::Color::green()));
        window_->showWidget("Camera GT", *camera_gt_);
        
        camera_.reset(new cv::viz::WCameraPosition(sim_setup->getVision().getCameraCalibrationMatrix(), 1, cv::viz::Color::yellow()));
        window_->showWidget("Camera", *camera_);
    
        global_vio_simulation_ = this;
        window_->registerKeyboardCallback(&VioSimulation::windowKeyboardCallback);
        
        window_->spinOnce();
    }
    
    global_vio_simulation_ = this;
    sim_setup->getImu().initialize(this);
    sim_setup->getVision().initialize(this);
    sim_setup->getTrajectory().initialize(this);
    sim_setup->getOdometry().initialize(this);
    
    is_simulation_running_ = true;
    std::thread t(&VioSimulation::startRunLoop, this);
    
    while (!window_->wasStopped()) {
        std::lock_guard<std::mutex> _(ui_lock_);
        window_->spinOnce();
    }
    run_loop_.stop();
    t.join();
}

void VioSimulation::accelerometerCallback(double time, Eigen::Vector3d accel) {
    sim_setup_->getOdometry().updateAcceleration(time, accel);
}

void VioSimulation::gyroscopeCallback(double time, Eigen::Vector3d gyro) {
    sim_setup_->getOdometry().updateRotationRate(time, gyro);
}

void VioSimulation::cameraCallback(double time, cv::Mat frame) {
    sim_setup_->getOdometry().updateFrame(time, frame);
}

void VioSimulation::runLoopCallback(double time) {
    Trajectory& trajectory = sim_setup_->getTrajectory();
    const Odometry& odometry = sim_setup_->getOdometry();
    const Vision& vision = sim_setup_->getVision();
    
    Eigen::Vector3d p_B_G_gt = trajectory.getCameraPositionInGlobalFrame(time);
    tonav::Quaternion q_G_B_gt = trajectory.getGlobalToCameraFrameRotation(time).conjugate();
    
    Eigen::Vector3d p_C_G = odometry.getCameraPositionInGlobalFrame();
    tonav::Quaternion q_G_C = odometry.getGlobalToCameraFrameRotation().conjugate();
    
    std::vector<cv::Affine3d::Vec3> features_in_view;
    for (const Eigen::Vector3d& p_f_G: vision.getFeaturesInView()) {
        cv::Affine3d pose = getPose(tonav::Quaternion::identity(), p_f_G);
        features_in_view.push_back(pose.translation());
    }
    
    {
        std::lock_guard<std::mutex> _(ui_lock_);
        window_->setWidgetPose("Camera GT", getPose(q_G_B_gt, p_B_G_gt));
        window_->setWidgetPose("Camera", getPose(q_G_C, p_C_G));
        
        if (features_cloud_) {
            window_->removeWidget("Features cloud");
            features_cloud_.reset();
        }
        if (!features_in_view.empty()) {
            features_cloud_.reset(new cv::viz::WCloud(features_in_view, cv::viz::Color::green()));
            features_cloud_->setRenderingProperty(cv::viz::POINT_SIZE, 3);
    
            window_->showWidget("Features cloud", *features_cloud_);
        }
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

void VioSimulation::windowKeyboardCallback(const cv::viz::KeyboardEvent& event, void*) {
    std::cout << "Window keyboard callback" << std::endl;
}

