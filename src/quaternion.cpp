#include "quaternion.h"

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

Quaternion::Quaternion(double x, double y, double z, double w) {
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
}

Quaternion::Quaternion(const Quaternion& other) {
    x_ = other.x_;
    y_ = other.y_;
    z_ = other.z_;
    w_ = other.w_;
}

Quaternion& Quaternion::operator=(const Quaternion &other) {
    x_ = other.x_;
    y_ = other.y_;
    z_ = other.z_;
    w_ = other.w_;
    return *this;
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
    double x = w_*rhs.x_ + z_*rhs.y_ - y_*rhs.z_ + x_*rhs.w_;
    double y = -1*z_*rhs.x_ + w_*rhs.y_ + x_*rhs.z_ + y_*rhs.w_;
    double z = y_*rhs.x_ - x_*rhs.y_ + w_*rhs.z_ + z_*rhs.w_;
    double w = -1*x_*rhs.x_ - y_*rhs.y_ - z_*rhs.z_ + w_*rhs.w_;
    return Quaternion(x, y, z, w);
}

double Quaternion::norm() const {
    return std::sqrt(x_*x_ + y_*y_ + z_*z_ + w_*w_);
}

void Quaternion::normalize() {
    double n = norm();
    x_ /= n;
    y_ /= n;
    z_ /= n;
    w_ /= n;
}

Quaternion Quaternion::normalized() const {
    Quaternion q(*this);
    q.normalize();
    return q;
}

bool Quaternion::isUnitQuaternion(double eps) const {
    return std::abs(norm() - 1.0) < eps;
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(-1*x_, -1*y_, -1*z_, w_);
}

Eigen::Matrix3d Quaternion::toRotationMatrix() const {
    Eigen::Matrix3d r = Eigen::Matrix3d::Zero();
    
    r(0, 0) = x_*x_ - y_*y_ - z_*z_ + w_*w_;
    r(0, 1) = 2.0*(x_*y_ + z_*w_);
    r(0, 2) = 2.0*(x_*z_ - y_*w_);
    
    r(1, 0) = 2.0*(x_*y_ - z_*w_);
    r(1, 1) = -1*x_*x_ + y_*y_ - z_*z_ + w_*w_;
    r(1, 2) = 2.0*(y_*z_ + x_*w_);
    
    r(2, 0) = 2.0*(x_*z_ + y_*w_);
    r(2, 1) = 2.0*(y_*z_ - x_*w_);
    r(2, 2) = -1*x_*x_ - y_*y_ + z_*z_ + w_*w_;
    
    return r;
}

double Quaternion::x() const {
    return x_;
}

double Quaternion::y() const {
    return y_;
}

double Quaternion::z() const {
    return z_;
}

double Quaternion::w() const {
    return w_;
}

Eigen::Vector3d Quaternion::vec() const {
    Eigen::Vector3d vec;
    vec << x_, y_, z_;
    return vec;
}

Eigen::Vector4d Quaternion::coeffs() const {
    Eigen::Vector4d coeffs;
    coeffs << x_, y_, z_, w_;
    return coeffs;
}

bool Quaternion::isApprox(const Quaternion& other, double eps) const {
    // Because q and -q represent the same rotation, we have to check is w is 1 or -1
    Quaternion q = *this * other.conjugate();
    bool x_close = std::abs(q.x()) < eps;
    bool y_close = std::abs(q.y()) < eps;
    bool z_close = std::abs(q.z()) < eps;
    bool w_close = std::abs(std::abs(q.w()) - 1.0) < eps;
    return x_close && y_close && z_close && w_close;
}

Quaternion Quaternion::identity() {
    return Quaternion(0.0, 0.0, 0.0, 1.0);
}

Quaternion Quaternion::fromRotationMatrix(const Eigen::Matrix3d& m) {
    Eigen::Quaterniond q(m.transpose());
    return Quaternion(q.x(), q.y(), q.z(), q.w());
}
