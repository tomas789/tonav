//
// Created by Tomas Krejci on 10/7/17.
//

#include "imu/numerical_diff_imu.h"

#include "sim_setup.h"
#include "vio_simulation.h"

std::unique_ptr<Imu> NumericalDiffImu::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<NumericalDiffImu> imu(new NumericalDiffImu(sim_setup));
    
    imu->diff_time_delta_ = j.at("diff_time_delta").get<double>();
    
    return std::move(imu);
}

void NumericalDiffImu::initialize(VioSimulation *simulation) {
    simulation_ = simulation;
    simulation_->getRunLoop().registerCallback(0, this);
}

void NumericalDiffImu::runLoopCallback(double time) {
    Eigen::Vector3d accel = getAccelerometerData(time);
    Eigen::Vector3d gyro = getGyroscopeData(time);
    
    simulation_->accelerometerCallback(time, accel);
    simulation_->gyroscopeCallback(time, gyro);
    
    simulation_->getRunLoop().registerCallback(time+1.0/update_frequency_, this);
}

Eigen::Vector3d NumericalDiffImu::getAccelerometerData(double time) const {
    std::function<Eigen::Vector3d(double)> f = [&](double t) {
        return getVelocity(t);
    };
    Eigen::Vector3d a_g = diff(f, time, diff_time_delta_);
    Eigen::Matrix3d R_B_G = sim_setup_->getTrajectory().getGlobalToBodyFrameRotation(time).toRotationMatrix();
    Eigen::Vector3d global_gravity = sim_setup_->getTrajectory().getGlobalGravity();
    Eigen::Vector3d a_b = R_B_G*(a_g - global_gravity);
    Eigen::Matrix3d T_a = Eigen::Matrix3d::Identity();
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d a_m = T_a*a_b + b_a;
    return a_m;
}

Eigen::Vector3d NumericalDiffImu::getGyroscopeData(double time) const {
    // https://math.stackexchange.com/questions/2282938/converting-from-quaternion-to-angular-velocity-then-back-to-quaternion
    const Trajectory& trajectory = sim_setup_->getTrajectory();
    tonav::Quaternion q_t = trajectory.getGlobalToBodyFrameRotation(time);
    tonav::Quaternion q_th = trajectory.getGlobalToBodyFrameRotation(time+diff_time_delta_);
    Eigen::Vector4d q_diff_vec = (q_th.coeffs() - q_t.coeffs())/diff_time_delta_;
    tonav::Quaternion q_diff(q_diff_vec(0), q_diff_vec(1), q_diff_vec(2), q_diff_vec(3));
    Eigen::Vector4d q_omega = 2*(q_t.conjugate()*q_diff).coeffs();
    return q_omega.block<3, 1>(0, 0);
}

NumericalDiffImu::~NumericalDiffImu() = default;

NumericalDiffImu::NumericalDiffImu(SimSetup *sim_setup) : Imu(sim_setup) {

}

Eigen::Vector3d NumericalDiffImu::getVelocity(double time) const {
    std::function<Eigen::Vector3d(double)> f = [&](double t) {
        return sim_setup_->getTrajectory().getBodyPositionInGlobalFrame(t);
    };
    return diff(f, time, diff_time_delta_);
}
