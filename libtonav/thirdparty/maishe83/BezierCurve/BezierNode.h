//
//  BezierNode.h
//  MSCKF
//
//  Created by Michael Shelley on 19/04/14.
//  Copyright (c) 2014 Michael Shelley. All rights reserved.
//

#ifndef MSCKF_BezierNode_h
#define MSCKF_BezierNode_h

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

template <class T>
class BezierNode {
};

/* *************************************************************************************************** */
// Vector3f Version
template <>
class BezierNode<Vector3f> {
    Vector3f v;
public:
    Vector3f value() const {
        return v;
    }

    // Constructors
    BezierNode<Eigen::Vector3f>() : v(Vector3f(0, 0, 0)) {}
    BezierNode<Eigen::Vector3f>(const Vector3f &vec) : v(vec) {}

    // Sum
    const BezierNode<Vector3f> operator +(const BezierNode<Vector3f> &rhs) const {
        Vector3f vec = this->v + rhs.v;
        return BezierNode<Vector3f>(vec);
    }

    // Diff
    const BezierNode<Vector3f> operator -(const BezierNode<Vector3f> &rhs) const {
        Vector3f vec = this->v - rhs.v;
        return BezierNode<Vector3f>(vec);
    }

    // Times
    const BezierNode<Vector3f> operator *(const float n) const {
        Vector3f vec = this->v * n;
        return BezierNode<Vector3f>(vec);
    }

    // Interpolate
    static const BezierNode<Vector3f> Interpolate(const BezierNode<Vector3f> &lhs, const BezierNode<Vector3f> &rhs, const float u) {
        Vector3f vec = lhs.v + (rhs.v - lhs.v) * u;
        return BezierNode<Vector3f>(vec);
    }

    template <class SclassT>
    friend std::ostream& operator<< (std::ostream & os, const BezierNode<SclassT>& rhs);
};

template <>
std::ostream& operator<< (std::ostream & os, const BezierNode<Vector3f>& rhs)
{
    os << rhs.v.transpose();
    return os;
}

/* *************************************************************************************************** */
// Quaternionf Version
template <>
class BezierNode<Quaternionf> {
    Quaternionf q;
public:
    Quaternionf value() const {
        return q;
    }

    // Constructors
    BezierNode<Eigen::Quaternionf>() : q(Quaternionf::Identity()) {}
    BezierNode<Eigen::Quaternionf>(const Quaternionf &quat) : q(quat) {}

    // Sum
    const BezierNode<Quaternionf> operator +(const BezierNode<Quaternionf> &rhs) const {
        Quaternionf quat = rhs.q * this->q;
        return BezierNode<Quaternionf>(quat);
    }

    // Diff
    const BezierNode<Quaternionf> operator -(const BezierNode<Quaternionf> &rhs) const {
        Quaternionf quat = this->q * rhs.q.inverse();
        return BezierNode<Quaternionf>(quat);
    }

    // Times
    const BezierNode<Quaternionf> operator *(const float n) const {

        // q^n = exp( ln( q ) * n );
        // q = [ v; w ];

        Quaternionf quat = this->q;
        Vector3f v = quat.coeffs().head(3);
        float v_norm = v.norm();
        float q_norm = quat.coeffs().norm();
        if (v_norm > 1e-8) {

            // ln(q) = [ v * acos(w / |q|) / |v|; ln(|q|) ];
            quat.coeffs() << v * acos(quat.w() / q_norm) / v_norm, log(q_norm);

            quat.coeffs() = quat.coeffs() * n;

            // exp(q) = exp(w) * [ v * sin(|v|) / |v|; cos(|v|) ];
            quat.coeffs() << exp(quat.w()) * v * sinf(v_norm) / v_norm, exp(quat.w()) * cosf(v_norm);

            quat.normalize();
        }

        return BezierNode<Quaternionf>(quat);
    }

    // Interpolate
    static const BezierNode<Quaternionf> Interpolate(const BezierNode<Quaternionf> &lhs, const BezierNode<Quaternionf> &rhs, const float u) {
        float theta = acosf(lhs.q.normalized().coeffs().transpose() * rhs.q.normalized().coeffs());
        if (theta == 0) {
            return lhs;
        }

        if (isnan(theta)) {
            return lhs;
        }

        Quaternionf q = Quaternionf(lhs.q.coeffs() * (sinf((1-u)*theta) / sinf(theta)) +
                                    rhs.q.coeffs() * sinf(u*theta) / sinf(theta));
        
        return BezierNode<Quaternionf>(q.normalized());
    }

    template <class SclassT>
    friend std::ostream& operator<< (std::ostream & os, const BezierNode<SclassT>& rhs);
};

template <>
std::ostream& operator<< (std::ostream & os, const BezierNode<Quaternionf>& rhs)
{
    os << rhs.q.coeffs().transpose();
    return os;
}




#endif
