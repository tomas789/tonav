//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_SIM_SETUP_H
#define TONAV_SIM_SETUP_H

#include "imu.h"
#include "odometry.h"
#include "trajectory.h"
#include "vision.h"

class SimSetup {
public:
    static std::shared_ptr<SimSetup> load(std::string filename);
    
    Trajectory& getTrajectory();
    const Trajectory& getTrajectory() const;
    
    Imu& getImu();
    const Imu& getImu() const;
    
    Vision& getVision();
    const Vision& getVision() const;
    
    Odometry& getOdometry();
    const Odometry& getOdometry() const;
    
private:
    SimSetup();
    
    std::unique_ptr<Trajectory> trajectory_;
    std::unique_ptr<Imu> imu_;
    std::unique_ptr<Vision> vision_;
    std::unique_ptr<Odometry> odometry_;
};

#endif //TONAV_SIM_SETUP_H
