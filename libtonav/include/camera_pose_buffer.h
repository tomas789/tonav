#ifndef TONAV_CAMERA_POSE_BUFFER_H
#define TONAV_CAMERA_POSE_BUFFER_H

#include <iterator>
#include <utility>
#include <type_traits>

#include "camera_pose.h"
#include "circular_buffer.hpp"

namespace tonav {

using CameraPoseBuffer = CircularBuffer<CameraPose>;

}

#endif //TONAV_CAMERA_POSE_BUFFER_H
