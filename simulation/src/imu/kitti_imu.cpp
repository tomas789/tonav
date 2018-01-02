#include "imu/kitti_imu.h"
#include "vio_simulation.h"

std::unique_ptr<Imu> KittiImu::load(SimSetup* sim_setup, const KittiLoaderHelper& helper) {
    std::unique_ptr<KittiImu> imu(new KittiImu(sim_setup));
    
    return imu;
}

void KittiImu::initialize(VioSimulation *simulation) {
    simulation_ = simulation;
    if (!timestamps_.empty()) {
        simulation_->getRunLoop().registerCallback(timestamps_.front(), this);
        timestamps_.pop_front();
    }
}

void KittiImu::runLoopCallback(double time) {
    Eigen::Vector3d accelerometer = accelerometer_[time];
    Eigen::Vector3d gyroscope = gyroscope_[time];
    
    simulation_->accelerometerCallback(time, accelerometer);
    simulation_->gyroscopeCallback(time, gyroscope);
    
    if (!timestamps_.empty()) {
        simulation_->getRunLoop().registerCallback(timestamps_.front(), this);
        timestamps_.pop_front();
    }
}

Eigen::Vector3d KittiImu::getAccelerometerData(double time) const {
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d KittiImu::getGyroscopeData(double time) const {
    return Eigen::Vector3d::Zero();
}

Eigen::Vector3d KittiImu::getVelocity(double time) const {
    return Eigen::Vector3d::Zero();
}

KittiImu::KittiImu(SimSetup* sim_setup) : Imu(sim_setup) { }
KittiImu::~KittiImu() = default;
