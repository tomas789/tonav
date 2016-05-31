//
// Created by Tomas Krejci on 5/11/16.
//

#include "camera_item.h"

#include "calibration.h"
#include "exceptions/general_exception.h"

CameraItem CameraItem::fromString(std::string line) {
    CameraItem item;
    std::size_t pos = 0;
    for (int i = 0; i < 2; ++i) {
        if (pos == std::string::npos) {
            throw GeneralException("Failed parse camera data: Expected 2 columns. Got less.");
        }
        std::size_t pos_end = line.find(';', pos);
        std::string s = line.substr(pos, pos_end - pos);
        switch (i) {
            case 0:
                if (!Calibration::tryParseDouble(s, item.time_)) {
                    throw GeneralException("Failed parse camera data: Unable to parse as double " + s);
                }
                break;
            case 1:
                item.file_name_ = s;
                break;
            default:
                throw GeneralException("Failed parse camera data: Expected 2 columns. Got more.");
        }
        pos = pos_end + 1;
    }
    return item;
}

double CameraItem::getTime() const {
    return time_;
}

std::string CameraItem::getFileName() const {
    return file_name_;
}
