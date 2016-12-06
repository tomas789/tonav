#ifndef TONAV_STATE_INITIALIZER_H
#define TONAV_STATE_INITIALIZER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief Provides values for initialization of body state
 *
 * It includes position, velocity and orientation initial values.
 */
class StateInitializer {
public:
    StateInitializer();
    StateInitializer(const StateInitializer& other) = default;
    
    StateInitializer& operator=(const StateInitializer& other) = default;
    
    void setOrientation(const Eigen::Quaterniond& orientation);
    Eigen::Quaterniond getOrientation() const;
    
    void setPosition(const Eigen::Vector3d& position);
    Eigen::Vector3d getPosition() const;
    
    void setVelocity(const Eigen::Vector3d& velocity);
    Eigen::Vector3d getVelocity() const;
    
private:
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
};

#endif //TONAV_STATE_INITIALIZER_H
