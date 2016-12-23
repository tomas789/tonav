//
// Created by Tomas Krejci on 2/12/16.
//

#ifndef TONAV_FEATURE_REZIDUALIZATION_RESULT_H
#define TONAV_FEATURE_REZIDUALIZATION_RESULT_H

#include <Eigen/Core>

class FeatureRezidualizationResult {
public:
    FeatureRezidualizationResult(std::size_t track_length, std::size_t poses_in_state);
    
    /**
     * @brief Validity indicator.
     *
     * @return true if feature was residualized without erores, false otherwise
     */
    operator bool() const;
    
    void setIsInvalid();
    
    /** @brief \f$r_{i,j}\f$ */
    void setPoseRezidual(std::size_t pose, const Eigen::Vector2d& rezidual);
    
    /** @brief \f$H_{c_{i,j}}\f$ */
    void setJacobianByCameraParameters(std::size_t pose, const Eigen::Matrix<double, 2, 14>& jacobian);
    
    /** @brief \f$H_{x_{i,B_j}}\f$ */
    void setJacobianByCameraPose(std::size_t pose, const Eigen::Matrix<double, 2, 9>& jacobian);
    
    /** @brief \f$H_{f_{i,j}}\f$ */
    void setJacobianByFeaturePosition(std::size_t pose, const Eigen::Matrix<double, 2, 3>& jacobian);
    
    Eigen::Vector3d getGlobalFeaturePosition() const;
    void setGlobalFeaturePosition(const Eigen::Vector3d& global_position);
    
    Eigen::VectorXd getReziduals() const;
    Eigen::MatrixXd getJacobianByState() const;
    Eigen::Matrix<double, Eigen::Dynamic, 3> getJacobianByFeaturePosition() const;
    
private:
    bool is_valid_;
    std::size_t track_length_;
    std::size_t poses_in_state_;
    Eigen::Vector3d global_position_;
    Eigen::VectorXd r_;
    Eigen::MatrixXd H_x_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> H_f_;
};

#endif //TONAV_FEATURE_REZIDUALIZATION_RESULT_H
