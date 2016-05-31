//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_GENERAL_EXCEPTION_H
#define TONAV_GENERAL_EXCEPTION_H

#include <string>

#include "exceptions/base_exception.h"

class GeneralException : public BaseException {
public:
    GeneralException(const std::string& msg);
    GeneralException(int line_number, const std::string& msg);

    virtual const char* what() const noexcept;

    virtual ~GeneralException();

private:
    std::string msg_;
};

#endif //TONAV_GENERAL_EXCEPTION_H
