#ifndef TONAV_TONAV_ROS_H
#define TONAV_TONAV_ROS_H

#include <boost/program_options.hpp>
#include <Eigen/Core>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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
    
    std::string robot_base_link_;
    std::string camera_frame_id_;
    std::string imu_frame_id_;
    
    int camera_callback_counter_ = 0;
    
    bool is_ready_to_filter_ = false;
    
    std::unique_ptr<image_transport::Publisher> image_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    void setAllowedOptionsDescription();
    bool parseCommandLineParams(int argc, char* argv[]);
    void printHelp();
    
    void cameraCallback(const sensor_msgs::ImageConstPtr& msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void twistCallback(const geometry_msgs::TwistStampedPtr& msg);
    double getMessageTime(ros::Time stamp);
    
    void publishResults(const ros::Time& time);
};

#endif //TONAV_TONAV_ROS_H
