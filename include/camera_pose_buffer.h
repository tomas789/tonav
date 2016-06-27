#ifndef TONAV_CAMERA_POSE_BUFFER_H
#define TONAV_CAMERA_POSE_BUFFER_H

#include <boost/circular_buffer.hpp>
#include <utility>

#include "camera_pose.h"

class CameraPoseBuffer {
public:
    using iterator = boost::circular_buffer<CameraPose>::iterator;
    
    CameraPoseBuffer(int max_camera_poses);
    
    void deleteOldestCameraPose();
    void addNewCameraPose(const CameraPose& pose);
    
    iterator begin();
    iterator end();
    
    CameraPose& front();
    CameraPose& back();
        
    std::size_t size() const;
    bool empty() const;
    
private:
    boost::circular_buffer<CameraPose> buffer_;
};

#endif //TONAV_CAMERA_POSE_BUFFER_H