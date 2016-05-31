//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_CAMERAFEED_H
#define TONAV_CAMERAFEED_H

#include <boost/filesystem/path.hpp>
#include <string>
#include <vector>

#include "camera_item.h"

namespace cv {
    class Mat;
}

class CameraFeed {
public:
    bool hasNext() const;
    void next();
    const CameraItem& top() const;

    cv::Mat getImage(const CameraItem& item) const;

    static CameraFeed fromDataset(boost::filesystem::path path);
private:
    void loadFile(boost::filesystem::path fpath);

    std::vector<CameraItem>::iterator current_item_;
    std::vector<CameraItem> items_;
    boost::filesystem::path path_;
};

#endif //TONAV_CAMERAFEED_H
