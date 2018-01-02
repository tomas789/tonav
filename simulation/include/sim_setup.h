//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_SIM_SETUP_H
#define TONAV_SIM_SETUP_H

#include "helper.h"
#include "imu.h"
#include "odometry.h"
#include "trajectory.h"
#include "vision.h"

class SimSetup {
public:
    static std::shared_ptr<SimSetup> load(std::string filename);
    static void registerDefaultComponents();
    
    bool hasHelper(const std::string& name) const;
    Helper& getHelper(const std::string& name);
    const Helper& getHelper(const std::string& name) const;
    std::vector<std::string> getAllHelperNames() const;
    
    Trajectory& getTrajectory();
    const Trajectory& getTrajectory() const;
    void setTrajectory(std::unique_ptr<Trajectory>&& trajectory);
    
    Imu& getImu();
    const Imu& getImu() const;
    void setImu(std::unique_ptr<Imu>&& imu);
    
    Vision& getVision();
    const Vision& getVision() const;
    void setVision(std::unique_ptr<Vision>&& vision);
    
    Odometry& getOdometry();
    const Odometry& getOdometry() const;
    void setOdometry(std::unique_ptr<Odometry>&& odometry);
    
    template <typename T>
    static void registerHelper() {
        SimSetup::registered_helper_.emplace(T::getComponentName(), T::load);
    }
    
    template <typename T>
    static void registerTrajectory() {
        SimSetup::registered_trajectory_.emplace(T::getComponentName(), T::load);
    }
    
    template <typename T>
    static void registerImu() {
        SimSetup::registered_imu_.emplace(T::getComponentName(), T::load);
    }
    
    template <typename T>
    static void registerVision() {
        SimSetup::registered_vision_.emplace(T::getComponentName(), T::load);
    }
    
    template <typename T>
    static void registerOdometry() {
        SimSetup::registered_odometry_.emplace(T::getComponentName(), T::load);
    }
    
private:
    SimSetup();
    
    template <typename T>
    using ComponentFactory = std::function<std::unique_ptr<T>(SimSetup*, const json&)>;
    
    static std::map<std::string, ComponentFactory<Helper>> registered_helper_;
    static std::map<std::string, ComponentFactory<Trajectory>> registered_trajectory_;
    static std::map<std::string, ComponentFactory<Imu>> registered_imu_;
    static std::map<std::string, ComponentFactory<Vision>> registered_vision_;
    static std::map<std::string, ComponentFactory<Odometry>> registered_odometry_;
    
    std::map<std::string, std::unique_ptr<Helper>> helpers_;
    std::unique_ptr<Trajectory> trajectory_;
    std::unique_ptr<Imu> imu_;
    std::unique_ptr<Vision> vision_;
    std::unique_ptr<Odometry> odometry_;
};

#endif //TONAV_SIM_SETUP_H
