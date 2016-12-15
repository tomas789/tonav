#ifndef TONAV_BASE_EXCEPTION_H
#define TONAV_BASE_EXCEPTION_H

#include <exception>

class BaseException : public std::exception {
public:
    BaseException();
    virtual const char* what() const noexcept = 0;
};

#endif //TONAV_BASE_EXCEPTION_H
