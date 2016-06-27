#include "camera_pose_buffer.h"

#include <iostream>
#include <boost/circular_buffer.hpp>
#include <utility>

#include "camera_pose.h"

CameraPoseBuffer::CameraPoseBuffer(int max_camera_poses)
: buffer_(max_camera_poses) {
}

void CameraPoseBuffer::deleteOldestCameraPose() {
    if (buffer_.empty()) {
        throw std::runtime_error("Trying to delete camera pose from empty CameraPoseBuffer");
    }
    buffer_.pop_front();
}

void CameraPoseBuffer::addNewCameraPose(const CameraPose& pose) {
    if (buffer_.full()) {
        throw std::runtime_error("CameraPoseBuffer is full. Cannot add another CameraPose.");
    }
    buffer_.push_back(pose);
}

CameraPoseBuffer::iterator CameraPoseBuffer::begin() {
    return buffer_.begin();
}

CameraPoseBuffer::iterator CameraPoseBuffer::end() {
    return buffer_.end();
}

CameraPose& CameraPoseBuffer::front() {
    if (buffer_.empty()) {
        throw std::runtime_error("CameraPoseBuffer is empty. Cannot return front element.");
    }
    return buffer_.front();
}

CameraPose& CameraPoseBuffer::back() {
    if (buffer_.empty()) {
        throw std::runtime_error("CameraPoseBuffer is empty. Cannot return back element.");
    }
    return buffer_.back();
}

std::size_t CameraPoseBuffer::size() const {
    return buffer_.size();
}

bool CameraPoseBuffer::empty() const {
    return buffer_.empty();
}