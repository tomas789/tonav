//
// Created by Tomas Krejci on 5/3/16.
//

#ifndef TONAV_IMPOSSIBLE_EXCEPTION_H
#define TONAV_IMPOSSIBLE_EXCEPTION_H

#include <string>

#include "exceptions/base_exception.h"

class ImpossibleException : public BaseException {
public:
    ImpossibleException(const std::string& msg);

    virtual const char* what() const noexcept;

    ~ImpossibleException();

private:
    std::string msg_;
};

#endif //TONAV_IMPOSSIBLE_EXCEPTION_H
