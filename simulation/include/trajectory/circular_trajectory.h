//
// Created by Tomas Krejci on 10/8/17.
//

#ifndef TONAV_CIRCULAR_TRAJECTORY_H
#define TONAV_CIRCULAR_TRAJECTORY_H

#include <json.hpp>
#include <opencv2/features2d.hpp>

#include "../trajectory.h"

class VioSimulation;

using json = nlohmann::json;

class CircularTrajectory: public Trajectory {
public:
    static std::unique_ptr<Trajectory> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    Eigen::Vector3d getBodyPositionInGlobalFrame(double time);
    tonav::Quaternion getGlobalToBodyFrameRotation(double time);
    
    Eigen::Vector3d getCameraPositionInGlobalFrame(double time);
    tonav::Quaternion getGlobalToCameraFrameRotation(double time);
    
    cv::Feature2D& getFeature2D();
    
    virtual ~CircularTrajectory();

protected:
    CircularTrajectory(SimSetup *sim_setup);
    
    class VirtualFeatures: public cv::Feature2D {
    public:
        VirtualFeatures(CircularTrajectory& trajectory);
        
        virtual ~VirtualFeatures();
        
    private:
        CircularTrajectory &trajectory_;
    };
    
    double radius_;
    double time_per_revolution_;
    
    tonav::Quaternion q_C_B_;
    Eigen::Vector3d p_C_B_;
    
    VirtualFeatures feature2d_;
};

#endif //TONAV_CIRCULAR_TRAJECTORY_H
