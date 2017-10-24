//
// Created by Tomas Krejci on 10/21/17.
//

#ifndef TONAV_CUSTOM_SERIALIZATION_H
#define TONAV_CUSTOM_SERIALIZATION_H

#include <json.hpp>
#include <Eigen/Dense>
#include <tonav.h>

namespace nlohmann {

template <>
struct adl_serializer<tonav::Quaternion> {
    static void to_json(json &j, const tonav::Quaternion &q) {
        j = json { {"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()} };
    }
    
    static void from_json(const json &j, tonav::Quaternion &q) {
        q = tonav::Quaternion(j.at("x"), j.at("y"), j.at("z"), j.at("w"));
    }
};

template <>
struct adl_serializer<Eigen::Vector2d> {
    static void to_json(json &j, const Eigen::Vector2d &v) {
        j = json { v(0), v(1) };
    }
    
    static void from_json(const json &j, Eigen::Vector2d &v) {
        v << j.at(0), j.at(1);
    }
};

template <>
struct adl_serializer<Eigen::Vector3d> {
    static void to_json(json &j, const Eigen::Vector3d &v) {
        j = json { v(0), v(1), v(2) };
    }

    static void from_json(const json &j, Eigen::Vector3d &v) {
        v << j.at(0), j.at(1), j.at(2);
    }
};

}

#endif //TONAV_CUSTOM_SERIALIZATION_H
