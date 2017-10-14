//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_IMU_H
#define TONAV_IMU_H

#include <json.hpp>
#include <Eigen/Core>
#include <memory>

#include "sim_setup_component.h"
#include "trajectory.h"

using json = nlohmann::json;

class Imu: protected SimSetupComponent {
public:
    static std::unique_ptr<Imu> load(SimSetup *sim_setup, const json& j);
    
    virtual void initialize(VioSimulation *simulation) = 0;
    
    float getUpdateFrequency() const;
    
    virtual Eigen::Vector3d getAccelerometerData(double time) const = 0;
    virtual Eigen::Vector3d getGyroscopeData(double time) const = 0;
    
    virtual ~Imu();
    
protected:
    Imu(SimSetup *sim_setup);
    
    double update_frequency_;
};

#endif //TONAV_IMU_H
