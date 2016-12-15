#include "camera_pose_buffer.h"

#include <boost/circular_buffer.hpp>
#include <stdexcept>
#include <utility>

#include "camera_pose.h"

CameraPoseBuffer::CameraPoseBuffer(int max_camera_poses)
: buffer_(max_camera_poses) {
}

CameraPose& CameraPoseBuffer::operator[](std::size_t i) {
    return buffer_[i];
}

const CameraPose& CameraPoseBuffer::operator[](std::size_t i) const {
    return buffer_[i];
}

void CameraPoseBuffer::deleteOldestCameraPose() {
    if (buffer_.empty()) {
        throw std::runtime_error("Trying to delete camera pose from empty CameraPoseBuffer");
    }
    buffer_.pop_front();
}

void CameraPoseBuffer::addNewCameraPose(CameraPose&& pose) {
    if (buffer_.full()) {
        throw std::runtime_error("CameraPoseBuffer is full. Cannot add another CameraPose.");
    }
    buffer_.push_back(std::move(pose));
}

CameraPoseBuffer::iterator CameraPoseBuffer::begin() {
    return buffer_.begin();
}

CameraPoseBuffer::iterator CameraPoseBuffer::end() {
    return buffer_.end();
}

CameraPoseBuffer::const_iterator CameraPoseBuffer::begin() const {
    return buffer_.begin();
}

CameraPoseBuffer::const_iterator CameraPoseBuffer::end() const {
    return buffer_.end();
}

CameraPoseBuffer::reverse_iterator CameraPoseBuffer::rbegin() noexcept {
    return buffer_.rbegin();
}

CameraPoseBuffer::reverse_iterator CameraPoseBuffer::rend() noexcept {
    return buffer_.rend();
}

CameraPoseBuffer::const_reverse_iterator CameraPoseBuffer::rbegin() const noexcept {
    return buffer_.rbegin();
}

CameraPoseBuffer::const_reverse_iterator CameraPoseBuffer::rend() const noexcept {
    return buffer_.rend();
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
