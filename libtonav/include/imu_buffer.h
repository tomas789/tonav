//
// Created by Tomas Krejci on 5/11/16.
//

#ifndef TONAV_IMU_BUFFER_H
#define TONAV_IMU_BUFFER_H

#include <list>

#include "imu_item.h"

class ImuBuffer {
public:
    using container_type = std::list<ImuItem>;
    using iterator = container_type::iterator;
    using const_iterator = container_type::const_iterator;
    
    iterator begin();
    iterator end();
    const_iterator begin() const;
    const_iterator end() const;
    
    ImuItem& front();
    const ImuItem& front() const;
    ImuItem& back();
    const ImuItem& back() const;
    
    bool empty() const;
    std::size_t size() const;
    
    void push_back(const ImuItem& item);
    void push_back(ImuItem&& item);
    
    void truncateToMinimalInterpolationTime(double time);
    
    static ImuItem interpolate(double time, const ImuItem& earlier, const ImuItem& later);
    static ImuItem interpolateAnyTime(double time, iterator hint);
    static void shiftToInterpolationInterval(double time, iterator& it);
private:
    container_type buffer_;
};

#endif //TONAV_IMU_BUFFER_H
