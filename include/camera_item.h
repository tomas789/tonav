//
// Created by Tomas Krejci on 5/10/16.
//

#ifndef TONAV_CAMERA_ITEM_H
#define TONAV_CAMERA_ITEM_H

#include <string>

class CameraItem {
public:
    static CameraItem fromString(std::string fname);

    double getTime() const;
    std::string getFileName() const;

private:
    double time_;
    std::string file_name_;
};

#endif //TONAV_CAMERA_ITEM_H
