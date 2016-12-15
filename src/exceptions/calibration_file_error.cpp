#include "exceptions/calibration_file_error.h"

#include <string>

CalibrationFileError::CalibrationFileError(const std::string& msg)
        : BaseException() {
    msg_ = msg;
}

CalibrationFileError::CalibrationFileError(int line_number, const std::string &msg)
        : BaseException() {
    msg_ = "Configuration file error at line " + std::to_string(line_number) + ": " + msg;
}

const char* CalibrationFileError::what() const noexcept {
    return msg_.c_str();
}

CalibrationFileError::~CalibrationFileError() {

}

