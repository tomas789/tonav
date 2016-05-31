//
// Created by Tomas Krejci on 5/10/16.
//

#include "imu_feed.h"

#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <iterator>

#include "exceptions/general_exception.h"
#include "imu_item.h"

bool ImuFeed::hasNext() const {
    return current_item_ != std::end(items_);
}

void ImuFeed::next() {
    std::advance(current_item_, 1);
}

const ImuItem& ImuFeed::top() const {
    if (!hasNext()) {
        throw GeneralException("Trying to access after the end of imu data stream.");
    }
    return *current_item_;
}

ImuFeed ImuFeed::fromDataset(boost::filesystem::path path) {
    ImuFeed feed;
    feed.loadFile(path / "inertial_device.csv");
    return feed;
}

void ImuFeed::loadFile(boost::filesystem::path fpath) {
    if (!boost::filesystem::exists(fpath)) {
        throw GeneralException("File not exists " + fpath.string());
    }
    if (!boost::filesystem::is_regular_file(fpath)) {
        throw GeneralException("Not an regular file " + fpath.string());
    }
    std::ifstream input_file(fpath.c_str());
    if (!input_file) {
        throw GeneralException("Not able to process IMU file.");
    }

    std::string header;
    std::getline(input_file, header);

    while (input_file) {
        std::string line;
        std::getline(input_file, line);
        if (line.empty()) {
            continue;
        }
        ImuItem item = ImuItem::fromString(line);
        items_.push_back(item);
    }

    current_item_ = std::begin(items_);
}