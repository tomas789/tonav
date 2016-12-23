//
// Created by Tomas Krejci on 5/10/16.
//

#include "exceptions/general_exception.h"

#include <string>

#include "exceptions/base_exception.h"

GeneralException::GeneralException(const std::string& msg)
        : BaseException() {
    msg_ = msg;
}

GeneralException::GeneralException(int line_number, const std::string &msg)
        : BaseException() {
    msg_ = "Configuration file error at line " + std::to_string(line_number) + ": " + msg;
}

const char* GeneralException::what() const noexcept {
    return msg_.c_str();
}

GeneralException::~GeneralException() {

}

