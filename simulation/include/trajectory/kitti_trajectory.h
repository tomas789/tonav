//
// Created by Tomas Krejci on 26/12/17.
//

#ifndef TONAV_KITTI_TRAJECTORY_H
#define TONAV_KITTI_TRAJECTORY_H

#include "../trajectory.h"

#include <map>

class VioSimulation;
class KittiLoaderHelper;

class KittiTrajectory: public Trajectory {
public:
    friend class KittiLoaderHelper;
    
    static std::unique_ptr<Trajectory> load(SimSetup* sim_setup, const KittiLoaderHelper& helper);
    
    void initialize(VioSimulation *simulation);
    
    Eigen::Vector3d getBodyPositionInGlobalFrame(double time) const;
    tonav::Quaternion getGlobalToBodyFrameRotation(double time) const;
    
    Eigen::Vector3d getGlobalGravity() const;
    
    virtual ~KittiTrajectory();
    
protected:
    KittiTrajectory(SimSetup *sim_setup);
    
    template <typename T>
    const T& findNearest(double time, const std::map<double, T>& map) const {
        auto lower_bound = map.lower_bound(time);
        typename std::map<double, T>::const_iterator nearest_it;
        if (lower_bound == std::end(map)) {
            nearest_it = std::prev(lower_bound);
        } else if (lower_bound == std::begin(map)) {
            nearest_it = lower_bound;
        } else {
            double dist_prev = time - std::prev(lower_bound)->first;
            double dist_lower_bound = lower_bound->first - time;
            if (dist_prev < dist_lower_bound) {
                nearest_it = std::prev(lower_bound);
            } else {
                nearest_it = lower_bound;
            }
        }
        return nearest_it->second;
    }
    
    std::map<double, tonav::Quaternion> q_B_G_gt_;
    std::map<double, Eigen::Vector3d> p_B_G_gt_;

};

#endif //TONAV_KITTI_TRAJECTORY_H
