#ifndef TONAV_BODY_STATE_H
#define TONAV_BODY_STATE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

class BodyState {
public:
    using BodyStateType = Eigen::Matrix<double, 16, 1>;
    
    Eigen::Block<BodyStateType, 4, 1> getRotationBlock();
    Eigen::Quaterniond getRotationQuaternion();
    void setRotationQuaternion(const Eigen::Quaterniond& quat);
    Eigen::Block<BodyStateType, 3, 1> getPositionBlock();
    Eigen::Block<BodyStateType, 3, 1> getVelocityBlock();
    Eigen::Block<BodyStateType, 3, 1> getAccelerometerBiasBlock();
    Eigen::Block<BodyStateType, 3, 1> getGyroscopeBiasBlock();
    
private:
    BodyStateType body_state_;
};

#endif //TONAV_BODY_STATE_H
