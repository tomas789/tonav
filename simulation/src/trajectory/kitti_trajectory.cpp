#include "trajectory/kitti_trajectory.h"

std::unique_ptr<Trajectory> KittiTrajectory::load(SimSetup *sim_setup, const KittiLoaderHelper &j) {
    std::unique_ptr<KittiTrajectory> trajectory(new KittiTrajectory(sim_setup));
    
    return trajectory;
}

void KittiTrajectory::initialize(VioSimulation *simulation) {
}

Eigen::Vector3d KittiTrajectory::getBodyPositionInGlobalFrame(double time) const {
    Eigen::Vector3d p_Bi_G_gt = findNearest(time, p_B_G_gt_);
    return p_Bi_G_gt;
}

tonav::Quaternion KittiTrajectory::getGlobalToBodyFrameRotation(double time) const {
    tonav::Quaternion q_Bi_G_gt = findNearest(time, q_B_G_gt_);
    return q_Bi_G_gt;
}

Eigen::Vector3d KittiTrajectory::getGlobalGravity() const {
    Eigen::Vector3d global_gravity;
    global_gravity << 0, 0, -9.81;
    return global_gravity;
}

KittiTrajectory::KittiTrajectory(SimSetup* sim_setup) : Trajectory(sim_setup) { }
KittiTrajectory::~KittiTrajectory() = default;
