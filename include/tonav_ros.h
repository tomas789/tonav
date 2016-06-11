#ifndef TONAV_TONAV_ROS_H
#define TONAV_TONAV_ROS_H

#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <boost/program_options.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tonav.h"

namespace po = boost::program_options;

/**
 * @brief Tonav navigation node for ROS.
 *
 * This is full implementation of ROS node performing navigation
 * using data directly from ROS. It also publishes result to ROS.
 */
class TonavRos {
public:
    TonavRos();
    
    /**
     * @brief Run navigation node.
     *
     * Run ROS node and start navigation. This is blocking function.
     * It also calls `ros::init` function.
     *
     * @param argc Number of command line arguments.
     * @param argv List of command line arguments.
     */
    int run(int argc, char* argv[]);
    
private:
    std::unique_ptr<Tonav> tonav_;
    ros::Time time_beginning_;
    
    po::options_description options_description_;
    po::variables_map variables_map_;
    
    Eigen::Matrix3d current_camera_matrix_;
    Eigen::Matrix<double, 5, 1> current_distirtion_params_;
    
    image_transport::Publisher image_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    void setAllowedOptionsDescription();
    bool parseCommandLineParams(int argc, char* argv[]);
    void printHelp();
    
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    double getMessageTime(ros::Time stamp);
    
    void publishResults();
};

#endif //TONAV_TONAV_ROS_H