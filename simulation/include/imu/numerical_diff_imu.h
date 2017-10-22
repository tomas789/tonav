//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_NUMERICAL_DIFF_IMU_H
#define TONAV_NUMERICAL_DIFF_IMU_H

#include "../imu.h"
#include "../run_loop_callback.h"

#include <json.hpp>

class VioSimulation;

using json = nlohmann::json;

class NumericalDiffImu: public Imu, public RunLoopCallback {
public:
    static std::unique_ptr<Imu> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    void runLoopCallback(double time);
    
    Eigen::Vector3d getAccelerometerData(double time) const;
    Eigen::Vector3d getGyroscopeData(double time) const;
    
    Eigen::Vector3d getVelocity(double time) const;
    
    virtual ~NumericalDiffImu();
    
protected:
    NumericalDiffImu(SimSetup *sim_setup);
    
    template <typename T>
    T diff(std::function<T(double)> f, double x, double h) const {
        auto p1 = -f(x+2.0*h);
        auto p2 = 8.0*f(x+h);
        auto p3 = -8.0*f(x-h);
        auto p4 = f(x-2.0*h);
        return (p1+p2+p3+p4)/(12.0*h);
    }
    
    VioSimulation *simulation_;
    
    double diff_time_delta_;
    
    Eigen::Vector3d global_gravity_;
};

#endif //TONAV_NUMERICAL_DIFF_IMU_H
