//
// Created by Tomas Krejci on 28/12/17.
//

#ifndef TONAV_KITTI_IMU_H
#define TONAV_KITTI_IMU_H

#include "../imu.h"
#include "../run_loop_callback.h"

#include <json.hpp>
#include <forward_list>

class VioSimulation;
class KittiLoaderHelper;

using json = nlohmann::json;

class KittiImu: public Imu, public RunLoopCallback {
public:
    friend class KittiLoaderHelper;
    
    static std::unique_ptr<Imu> load(SimSetup* sim_setup, const KittiLoaderHelper& helper);
    
    void initialize(VioSimulation *simulation);
    
    void runLoopCallback(double time);
    
    Eigen::Vector3d getAccelerometerData(double time) const;
    Eigen::Vector3d getGyroscopeData(double time) const;
    
    Eigen::Vector3d getVelocity(double time) const;
    
    virtual ~KittiImu();
    
protected:
    KittiImu(SimSetup *sim_setup);
    
    VioSimulation *simulation_;
    std::forward_list<double> timestamps_;
    std::map<double, Eigen::Vector3d> accelerometer_;
    std::map<double, Eigen::Vector3d> gyroscope_;
};

#endif //TONAV_KITTI_IMU_H

