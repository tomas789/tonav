#ifndef TONAV_QUATERNION_H
#define TONAV_QUATERNION_H

#include <Eigen/Core>

/**
 * @brief Quaternion using JPL notation
 *
 * This class mimics some basic functionality of Eigen::Quaterniond. They should both work
 * as a drop-in replacements for features used in Tonav, except for notations.
 */
class Quaternion {
public:
    Quaternion(double x, double y, double z, double w);
    Quaternion(const Quaternion& other);
    
    Quaternion& operator=(const Quaternion& other);
    Quaternion operator*(const Quaternion& rhs) const;
    
    double norm() const;
    void normalize();
    Quaternion normalized() const;
    bool isUnitQuaternion(double eps = 1e-12) const;
    
    Quaternion conjugate() const;
    Eigen::Matrix3d toRotationMatrix() const;
    
    double x() const;
    double y() const;
    double z() const;
    double w() const;
    Eigen::Vector3d vec() const;
    Eigen::Vector4d coeffs() const;
    
    bool isApprox(const Quaternion& other, double eps = 1e-12) const;
    
    static Quaternion identity();
    static Quaternion fromRotationMatrix(const Eigen::Matrix3d& m);
private:
    double x_;
    double y_;
    double z_;
    double w_;
};

#endif //TONAV_QUATERNION_H
