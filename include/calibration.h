//
// Created by Tomas Krejci on 5/3/16.
//

#ifndef TONAV_CALIBRATION_H
#define TONAV_CALIBRATION_H

#include <boost/filesystem/path.hpp>
#include <Eigen/Core>
#include <string>
#include <vector>


class Calibration {
public:
    int getMaxCameraPoses() const;

    Eigen::Matrix<double, 3, 3> getGSensitivityMatrix() const;
    Eigen::Matrix<double, 3, 3> getGyroscopeShapeMatrix() const;
    Eigen::Matrix<double, 3, 3> getAccelerometerShapeMatrix() const;

    Eigen::Matrix<double, 3, 1> getCameraToBodyOffset() const;

    double getFocalLengthX() const;
    double getFocalLengthY() const;
    double getOpticalCenterX() const;
    double getOpticalCenterY() const;

    Eigen::Matrix<double, 3, 1> getRadialDistortionParameters() const;
    Eigen::Matrix<double, 2, 1> getTangentialDistortionParameters() const;

    double getCameraDelayTime() const;
    double getCameraReadoutTime() const;

    static Calibration fromPath(boost::filesystem::path fname);

    static bool tryParseInt(const std::string& value, int& out);
    static bool tryParseDouble(const std::string& value, double& out);
    static bool tryParseMatrix3d(const std::string& value, Eigen::Matrix3d& out);
private:
    int max_camera_poses_;

    double f_x_;
    double f_y_;
    double c_x_;
    double c_y_;
    double p_c_b_x_;
    double p_c_b_y_;
    double p_c_b_z_;
    double t_d_;
    double t_r_;

    double k_1_;
    double k_2_;
    double k_3_;
    double p_1_;
    double p_2_;

    int orb_nfeatures_;

    Eigen::Matrix3d t_s_;
    Eigen::Matrix3d t_a_;
    Eigen::Matrix3d t_g_;

    static const std::vector<std::string> allowed_params_;
};


#endif //TONAV_CALIBRATION_H
