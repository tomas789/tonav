//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_VIO_SIMULATION_H
#define TONAV_VIO_SIMULATION_H

#include <vector>
#include <opencv2/viz.hpp>
#include <thread>
#include <mutex>
#include <tonav.h>

#include "sim_setup.h"
#include "run_loop.h"

class VioSimulation {
public:
    VioSimulation();
    
    void run(std::shared_ptr<SimSetup> sim_setup);
    
    void accelerometerCallback(double time, Eigen::Vector3d accel);
    void gyroscopeCallback(double time, Eigen::Vector3d gyro);
    void cameraCallback(double time, cv::Mat frame);
    void runLoopCallback(double time);
    
    RunLoop& getRunLoop();
    const RunLoop& getRunLoop() const;
    
private:
    void startRunLoop();
    
    cv::Affine3d getPose(tonav::Quaternion q, Eigen::Vector3d p) const;
    
    std::shared_ptr<SimSetup> sim_setup_;
    std::atomic<bool> is_simulation_running_;
    
    std::mutex ui_lock_;
    RunLoop run_loop_;
    
    tonav::Quaternion q_viz_sim_ = tonav::Quaternion(0.5, -0.5, 0.5, -0.5).conjugate();
    
    std::shared_ptr<cv::viz::Viz3d> window_;
    std::shared_ptr<cv::viz::WCameraPosition> camera_gt_;
    std::shared_ptr<cv::viz::WCameraPosition> camera_;
};

#endif //TONAV_VIO_SIMULATION_H
