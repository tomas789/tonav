//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_IMU_FEED_H
#define TONAV_IMU_FEED_H

#include <boost/filesystem/path.hpp>
#include <string>
#include <vector>

#include "imu_item.h"

class ImuFeed {
public:
    bool hasNext() const;
    void next();
    const ImuItem& top() const;

    static ImuFeed fromDataset(boost::filesystem::path path);

private:
    void loadFile(boost::filesystem::path fpath);

    std::vector<ImuItem>::iterator current_item_;
    std::vector<ImuItem> items_;
};

#endif //TONAV_IMU_FEED_H
