//
// Created by Tomas Krejci on 5/3/16.
//

#include "exceptions/impossible_exception.h"

#include <exception>
#include <string>

ImpossibleException::ImpossibleException(const std::string& msg) {
    msg_ = msg;
}

const char* ImpossibleException::what() const noexcept {
    return msg_.c_str();
}

ImpossibleException::~ImpossibleException() { }
