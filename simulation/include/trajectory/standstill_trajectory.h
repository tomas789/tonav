//
// Created by Tomas Krejci on 10/24/17.
//

#ifndef TONAV_STANDSTILL_TRAJECTORY_H
#define TONAV_STANDSTILL_TRAJECTORY_H

#include "../trajectory.h"

class StandstillTrajectory: public Trajectory {
public:
    static std::unique_ptr<StandstillTrajectory> load(SimSetup *sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    Eigen::Vector3d getBodyPositionInGlobalFrame(double time) const;
    tonav::Quaternion getGlobalToBodyFrameRotation(double time) const;
    
    Eigen::Vector3d getGlobalGravity() const;
    
    virtual ~StandstillTrajectory();
    
private:
    StandstillTrajectory(SimSetup *sim_setup);
};

#endif //TONAV_STANDSTILL_TRAJECTORY_H
