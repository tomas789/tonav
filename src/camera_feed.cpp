//
// Created by Tomas Krejci on 5/10/16.
//

#include "camera_feed.h"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iterator>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

#include "exceptions/general_exception.h"
#include "camera_item.h"

bool CameraFeed::hasNext() const {
    return current_item_ != std::end(items_);
}

void CameraFeed::next() {
    std::advance(current_item_, 1);
}

const CameraItem& CameraFeed::top() const {
    if (!hasNext()) {
        throw GeneralException("Trying to access after the end of camera data stream.");
    }
    return *current_item_;
}

CameraFeed CameraFeed::fromDataset(boost::filesystem::path path) {
    CameraFeed feed;
    feed.loadFile(path / "camera.csv");
    feed.path_ = path;
    return feed;
}

void CameraFeed::loadFile(boost::filesystem::path fpath) {
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
        CameraItem item = CameraItem::fromString(line);
        items_.push_back(item);
    }

    current_item_ = std::begin(items_);
}

cv::Mat CameraFeed::getImage(const CameraItem &item) const {
    boost::filesystem::path fpath = path_ / item.getFileName();
    cv::Mat mat;
    mat = cv::imread(fpath.string(), CV_LOAD_IMAGE_GRAYSCALE);
    if(mat.empty())
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return cv::Mat();
    }
    std::cout << "Shape: " << mat.size() << std::endl;
    std::cout << "Image: " << fpath.string() << std::endl;
    return mat;
}


