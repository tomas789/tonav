//
// Created by Tomas Krejci on 5/3/16.
//

#ifndef TONAV_CALIBRATION_H
#define TONAV_CALIBRATION_H

#include <Eigen/Core>
#include <string>
#include <vector>
#include <memory>

#include "quaternion.h"

namespace tonav {

/**
 * Read calibration file.
 *
 * @todo Add comments to all methods.
 */
class Calibration {
public:
    Calibration();
    
    Calibration(const Calibration &other) = default;
    
    Calibration &operator=(const Calibration &other) = default;
    
    void setCameraFocalLength(const Eigen::Vector2d &focal_length);
    
    Eigen::Vector2d getCameraFocalLength() const;
    
    void setCameraOpticalCenter(const Eigen::Vector2d &optical_center);
    
    Eigen::Vector2d getCameraOpticalCenter() const;
    
    void setCameraRadialDistortionParams(const Eigen::Vector3d &distortion_params);
    
    Eigen::Vector3d getCameraRadialDistortionParams() const;
    
    void setCameraTangentialDistortionParams(const Eigen::Vector2d &distortion_params);
    
    Eigen::Vector2d getCameraTangentialDistortionParams() const;
    
    void setCameraDelayTime(double delay_time);
    
    double getCameraDelayTime() const;
    
    void setCameraReadoutTime(double readout_time);
    
    double getCameraReadoutTime() const;
    
    void setImageNoiseVariance(double variance);
    
    double getImageNoiseVariance() const;
    
    int getNumberOfFeaturesToExtract() const;
    
    void setGyroscopeAccelerationSensitivityMatrix(const Eigen::Matrix3d& gyroscope_acceleration_sensitivity_matrix);
    
    Eigen::Matrix3d getGyroscopeAccelerationSensitivityMatrix() const;
    
    void setGyroscopeShapeMatrix(const Eigen::Matrix3d& gyroscope_shape_matrix);
    
    Eigen::Matrix3d getGyroscopeShapeMatrix() const;
    
    void setAccelerometerShapeMatrix(const Eigen::Matrix3d& accelerometer_shape_matrix);
    
    Eigen::Matrix3d getAccelerometerShapeMatrix() const;
    
    void setGyroscopeBias(const Eigen::Vector3d& gyroscope_bias);
    
    Eigen::Vector3d getGyroscopeBias() const;
    
    void setAccelerometerBias(const Eigen::Vector3d& accelerometer_bias);
    
    Eigen::Vector3d getAccelerometerBias() const;
    
    void setGlobalGravity(const Eigen::Vector3d& global_gravity);
    
    Eigen::Vector3d getGlobalGravity() const;
    
    void setAccelerometerVariance(double accelerometer_variance);
    
    double getAccelerometerVariance() const;
    
    void setGyroscopeVariance(double gyroscope_variance);
    
    double getGyroscopeVariance() const;
    
    void setAccelerometerRandomWalkVariance(double accelerometer_random_walk_variance);
    
    double getAccelerometerRandomWalkVariance() const;
    
    void setGyroscopeRandomWalkVariance(double gyroscope_random_walk_variance);
    
    double getGyroscopeRandomWalkVariance() const;
    
    void setMaxCameraPoses(int max_camera_poses);
    
    int getMaxCameraPoses() const;
    
    void setMaxTriangulationIterations(int max_triangulation_iterations);
    
    int getMaxTriangulationIterations() const;
    
    void setOrientationNoise(const Eigen::Vector3d& orientation_noise);
    
    Eigen::Vector3d getOrientationNoise() const;
    
    void setPositionNoise(const Eigen::Vector3d& position_noise);
    
    Eigen::Vector3d getPositionNoise() const;
    
    void setVelocityNoise(const Eigen::Vector3d& velocity_noise);
    
    Eigen::Vector3d getVelocityNoise() const;
    
    void setGyroscopeBiasNoise(const Eigen::Vector3d& gyroscope_bias_noise);
    
    Eigen::Vector3d getGyroscopeBiasNoise() const;
    
    void setAccelerometerBiasNoise(const Eigen::Vector3d& accelerometer_bias_noise);
    
    Eigen::Vector3d getAccelerometerBiasNoise() const;
    
    void setGyroscopeAccelerationSensitivityMatrixNoise(const Eigen::Matrix3d& gyroscope_acceleration_sensitivity_matrix_noise);
    
    Eigen::Matrix3d getGyroscopeAccelerationSensitivityMatrixNoise() const;
    
    void setGyroscopeShapeMatrixNoise(const Eigen::Matrix3d& gyroscope_shape_matrix_noise);
    
    Eigen::Matrix3d getGyroscopeShapeMatrixNoise() const;
    
    void setAccelerometerShapeMatrixNoise(const Eigen::Matrix3d& accelerometer_shape_matrix_noise);
    
    Eigen::Matrix3d getAccelerometerShapeMatrixNoise() const;
    
    void setPositionOfBodyInCameraFrameNoise(const Eigen::Vector3d& position_of_body_in_camera_frame_noise);
    
    Eigen::Vector3d getPositionOfBodyInCameraFrameNoise() const;
    
    void setFocalLengthNoise(const Eigen::Vector2d& focal_length_noise);
    
    Eigen::Vector2d getFocalLengthNoise() const;
    
    void setOpticalCenterNoise(const Eigen::Vector2d& optical_center_noise);
    
    Eigen::Vector2d getOpticalCenterNoise() const;
    
    void setRadialDistortionNoise(const Eigen::Vector3d& radial_distortion_noise);
    
    Eigen::Vector3d getRadialDistortionNoise() const;
    
    void setTangentialDistortionNoise(const Eigen::Vector2d& tangential_distortion_noise);
    
    Eigen::Vector2d getTangentialDistortionNoise() const;
    
    void setCameraDelayTimeNoise(double camera_delay_time_noise);
    
    double getCameraDelayTimeNoise() const;
    
    void setCameraReadoutTimeNoise(double camera_readout_time_noise);
    
    double getCameraReadoutTimeNoise() const;
    
    void setBodyToCameraRotation(const Quaternion &body_to_camera_rotation);
    
    Quaternion getBodyToCameraRotation() const;
    
    static std::shared_ptr<Calibration> fromPath(std::string fname);
    
    ~Calibration() = default;

protected:
    static bool tryParseInt(const std::string &value, int &out);
    
    static bool tryParseDouble(const std::string &value, double &out);
    
    static bool tryParseVector2d(const std::string &value, Eigen::Vector2d &out);
    
    static bool tryParseVector3d(const std::string &value, Eigen::Vector3d &out);
    
    static bool tryParseMatrix3d(const std::string &value, Eigen::Matrix3d &out);
    
    static bool tryParseString(const std::string &value, std::string &out);
    
    Eigen::Vector2d focal_length_;
    Eigen::Vector2d optical_center_;
    Eigen::Vector3d radial_distortion_;
    Eigen::Vector2d tangential_distortion_;
    double camera_delay_time_;
    double camera_readout_time_;
    double image_noise_variance_;
    
    int n_features_to_extract_;
    
    Eigen::Matrix3d gyroscope_acceleration_sensitivity_matrix_;
    Eigen::Matrix3d gyroscope_shape_matrix_;
    Eigen::Matrix3d accelerometer_shape_matrix_;
    Eigen::Vector3d gyroscope_bias_;
    Eigen::Vector3d accelerometer_bias_;
    Eigen::Vector3d global_gravity_;
    
    double accelerometer_variance_;
    double gyroscope_variance_;
    double accelerometer_random_walk_variance_;
    double gyroscope_random_walk_variance_;
    
    int max_camera_poses_;
    int max_triangulation_iterations_;
    
    Eigen::Vector3d orientation_noise_;
    Eigen::Vector3d position_noise_;
    Eigen::Vector3d velocity_noise_;
    Eigen::Vector3d gyroscope_bias_noise_;
    Eigen::Vector3d accelerometer_bias_noise_;
    Eigen::Matrix3d gyroscope_acceleration_sensitivity_matrix_noise_;
    Eigen::Matrix3d gyroscope_shape_matrix_noise_;
    Eigen::Matrix3d accelerometer_shape_matrix_noise_;
    Eigen::Vector3d position_of_body_in_camera_frame_noise_;
    Eigen::Vector2d focal_length_noise_;
    Eigen::Vector2d optical_center_noise_;
    Eigen::Vector3d radial_distortion_noise_;
    Eigen::Vector2d tangential_distortion_noise_;
    double camera_delay_time_noise_;
    double camera_readout_time_noise_;
    
    Quaternion body_to_camera_rotation_;
    
    static const std::vector<std::string> allowed_params_;
};

}

#endif //TONAV_CALIBRATION_H
