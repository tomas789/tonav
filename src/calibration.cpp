//
// Created by Tomas Krejci on 5/3/16.
//

#include "calibration.h"

#include <boost/filesystem.hpp>
#include <cctype>
#include <fstream>
#include <map>
#include <string>
#include <stdexcept>
#include <vector>
#include <exceptions/impossible_exception.h>
#include <iostream>

#include "exceptions/calibration_file_error.h"

const std::vector<std::string> Calibration::allowed_params_ = {
        "Camera.fx", "Camera.fy", "Camera.cx", "Camera.cy",
        "Camera.k1", "Camera.k2", "Camera.k3", "Camera.p1", "Camera.p2",
        "Camera.posToImuX", "Camera.posToImuY", "Camera.posToImuZ",
        "Camera.td", "Camera.tr",
        "ORBextractor.nFeatures",
        "Imu.Ts", "Imu.Tg", "Imu.Ta",
        "Filter.maxCameraPoses"
};

Calibration Calibration::fromPath(boost::filesystem::path fname) {
    Calibration calib;

    std::ifstream file(fname.c_str());
    if (!file) {
        throw std::runtime_error("Failed to open configuration file.");
    }

    int line_counter = 1;
    std::string line;
    std::map<std::string, std::string> params;
    std::map<std::string, int> param_loc;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            line_counter += 1;
            continue;
        }

        std::size_t delim_pos = line.find_first_of(":");
        if (delim_pos == std::string::npos) {
            throw CalibrationFileError(line_counter, "Missing delimiter.");
        }

        std::string key = line.substr(0, delim_pos);
        std::string value = line.substr(delim_pos + 1, std::string::npos);
        if (params.find(key) != std::end(params)) {
            throw CalibrationFileError(line_counter, "Value already set.");
        }
        if (std::find(std::begin(allowed_params_), std::end(allowed_params_), key) == std::end(allowed_params_)) {
            throw CalibrationFileError(line_counter, "Unknown parameter '" + key + "'.");
        }
        params[key] = value;
        param_loc[key] = line_counter;
        line_counter += 1;
    }
    for (const std::string& allowed_param : allowed_params_) {
        if (params.find(allowed_param) == std::end(params)) {
            throw CalibrationFileError("Parameter " + allowed_param + " not set in configuration file.");
        }
    }

    for (const std::pair<const std::string, std::string>& item : params) {
        std::string key = item.first;
        std::string value = item.second;
        if (key == "Camera.fx") {
            if (!Calibration::tryParseDouble(value, calib.f_x_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.fy") {
            if (!Calibration::tryParseDouble(value, calib.f_y_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.cx") {
            if (!Calibration::tryParseDouble(value, calib.c_x_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.cy") {
            if (!Calibration::tryParseDouble(value, calib.c_y_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.k1") {
            if (!Calibration::tryParseDouble(value, calib.k_1_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.k2") {
            if (!Calibration::tryParseDouble(value, calib.k_2_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.k3") {
            if (!Calibration::tryParseDouble(value, calib.k_3_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.p1") {
            if (!Calibration::tryParseDouble(value, calib.p_1_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.p2") {
            if (!Calibration::tryParseDouble(value, calib.p_2_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.posToImuX") {
            if (!Calibration::tryParseDouble(value, calib.p_c_b_x_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.posToImuY") {
            if (!Calibration::tryParseDouble(value, calib.p_c_b_y_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.posToImuZ") {
            if (!Calibration::tryParseDouble(value, calib.p_c_b_z_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.td") {
            if (!Calibration::tryParseDouble(value, calib.t_d_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Camera.tr") {
            if (!Calibration::tryParseDouble(value, calib.t_r_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "ORBextractor.nFeatures") {
            if (!Calibration::tryParseInt(value, calib.orb_nfeatures_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.Ts") {
            if (!Calibration::tryParseMatrix3d(value, calib.t_s_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.Tg") {
            if (!Calibration::tryParseMatrix3d(value, calib.t_g_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Imu.Ta") {
            if (!Calibration::tryParseMatrix3d(value, calib.t_a_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else if (key == "Filter.maxCameraPoses") {
            if (!Calibration::tryParseInt(value, calib.max_camera_poses_)) {
                throw CalibrationFileError(param_loc[key], "Unable to parse value.");
            }
        } else {
            throw ImpossibleException("Non-exhaustive enumeration in camera calibration loading.");
        }
    }

    return calib;
}

bool Calibration::tryParseDouble(const std::string &value, double &out) {
    try {
        std::size_t end = 0;
        out = std::stod(value, &end);
        for (std::size_t pos = end; pos < value.size(); ++pos) {
            if (!std::isspace(value[pos])) {
                return false;
            }
        }
        return true;
    } catch(...) {
        return false;
    }
}

bool Calibration::tryParseInt(const std::string &value, int &out) {
    try {
        std::size_t end = 0;
        out = std::stoi(value, &end);
        for (std::size_t pos = end; pos < value.size(); ++pos) {
            if (!std::isspace(value[pos])) {
                return false;
            }
        }
        return true;
    } catch(...) {
        return false;
    }
}

bool Calibration::tryParseMatrix3d(const std::string &value, Eigen::Matrix3d &out) {
    enum class Expect { OPEN, MAT_ITEM, LINE_SEP, CLOSE, ONLY_WHITESPACE };
    out = Eigen::Matrix3d::Zero();
    Expect expect = Expect::OPEN;
    int item_counter = 0;
    for (std::string::const_iterator it = std::begin(value); it != std::end(value); ++it) {
        if (std::isspace(*it)) {
            continue;
        }
        switch (expect) {
            case Expect::OPEN:
                if (*it != '[')
                    return false;
                expect = Expect::MAT_ITEM;
                break;
            case Expect::MAT_ITEM:
                try {
                    std::size_t after_pos = 0;
                    double item_parsed = stod(std::string(it, std::end(value)), &after_pos);
                    out(item_counter / 3, item_counter % 3) = item_parsed;
                    it += after_pos - 1;
                    item_counter += 1;
                } catch(...) {
                    return false;
                }
                if (item_counter == 3 || item_counter == 6) {
                    expect = Expect::LINE_SEP;
                } else if (item_counter == 9) {
                    expect = Expect::CLOSE;
                }
                break;
            case Expect::LINE_SEP:
                if (*it != ';')
                    return false;
                expect = item_counter == 9 ? Expect::CLOSE : Expect::MAT_ITEM;
                break;
            case Expect::CLOSE:
                if (*it != ']')
                    return false;
                expect = Expect::ONLY_WHITESPACE;
                break;
            case Expect::ONLY_WHITESPACE:
                return false;
                break;
            default:
                throw ImpossibleException("Calibration::tryParseMatrix3d: Non-comprehensive switch statement.");
                break;
        }
    }

    return true;
}

Eigen::Matrix<double, 3, 3> Calibration::getGSensitivityMatrix() const {
    return t_s_;
}

Eigen::Matrix<double, 3, 3> Calibration::getGyroscopeShapeMatrix() const {
    return t_g_;
}

Eigen::Matrix<double, 3, 3> Calibration::getAccelerometerShapeMatrix() const {
    return t_a_;
}

Eigen::Matrix<double, 3, 1> Calibration::getCameraToBodyOffset() const {
    Eigen::Matrix<double, 3, 1, 0, 3, 1> offset;
    offset << p_c_b_x_, p_c_b_y_, p_c_b_z_;
    return offset;
}

double Calibration::getFocalLengthX() const {
    return f_x_;
}

double Calibration::getFocalLengthY() const {
    return f_y_;
}

double Calibration::getOpticalCenterX() const {
    return c_x_;
}

double Calibration::getOpticalCenterY() const {
    return c_y_;
}

Eigen::Matrix<double, 3, 1> Calibration::getRadialDistortionParameters() const {
    Eigen::Matrix<double, 3, 1> params;
    params << k_1_, k_2_, k_3_;
    return params;
}

Eigen::Matrix<double, 2, 1> Calibration::getTangentialDistortionParameters() const {
    Eigen::Matrix<double, 2, 1> params;
    params << p_1_, p_2_;
    return params;
}

double Calibration::getCameraDelayTime() const {
    return t_d_;
}

double Calibration::getCameraReadoutTime() const {
    return t_r_;
}

int Calibration::getMaxCameraPoses() const {
    return max_camera_poses_;
}
























