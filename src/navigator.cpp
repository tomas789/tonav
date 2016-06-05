//
// Created by Tomas Krejci on 5/11/16.
//

#include "navigator.h"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "calibration.h"
#include "camera_feed.h"
#include "exceptions/base_exception.h"
#include "exceptions/impossible_exception.h"
#include "filter.h"
#include "feature_tracker.h"
#include "imu_buffer.h"
#include "imu_feed.h"

int Navigator::run(int argc, const char* argv[]) {
    namespace fs = boost::filesystem;

    parseOptions(argc, argv);

    if (vmap_.count("help")) {
        printHelp(false);
        return 0;
    }

    fs::path dataset_path(vmap_["dataset"].as<std::string>());
    fs::path calibration_path(vmap_["calib"].as<std::string>());

    if (!fs::exists(dataset_path)) {
        std::cerr << "Dataset path " << dataset_path.string() << " does not exists." << std::endl;
        printHelp(true);
        return 1;
    }

    try {
        Calibration calibration = Calibration::fromPath(calibration_path);
        Filter filter(calibration);
        filter.initialize();
        ImuFeed imu_feed = ImuFeed::fromDataset(dataset_path);
        CameraFeed camera_feed = CameraFeed::fromDataset(dataset_path);
        FeatureTracker feature_tracker;

        ros::init(argc, const_cast<char**>(argv), "image_publisher");
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("camera/image", 1);
        tf2_ros::TransformBroadcaster br;

        std::size_t buff_size = 100;
        ImuBuffer accel_buff(ImuDevice::ACCELEROMETER, buff_size);
        ImuBuffer gyro_buff(ImuDevice::GYROSCOPE, buff_size);

        double last_imu_update_time = 0.0;

        while (camera_feed.hasNext() && imu_feed.hasNext()) {
            if (imu_feed.top().getTime() <= camera_feed.top().getTime()) {
                // Propagate IMU
                const ImuItem& imu_item = imu_feed.top();
                imu_feed.next();
                std::cout.precision(5);
                std::cout << "Propagete IMU at time " << std::fixed << imu_item.getTime() << std::endl;

                switch (imu_item.getDevice()) {
                    case ImuDevice::ACCELEROMETER:
                        accel_buff.addMeasurement(imu_item);
                        break;
                    case ImuDevice::GYROSCOPE:
                        gyro_buff.addMeasurement(imu_item);
                        break;
                    default:
                        throw ImpossibleException("Unknown IMU device.");
                }

                if (!accel_buff.isReady() || !gyro_buff.isReady()) {
                    last_imu_update_time = imu_item.getTime();
                    continue;
                }

                bool accel_can_update = accel_buff.getMaxTime() > last_imu_update_time;
                bool gyro_can_update = gyro_buff.getMaxTime() > last_imu_update_time;
                if (accel_can_update && gyro_can_update) {
                    double update_time = std::min(accel_buff.getMaxTime(), gyro_buff.getMaxTime());
                    ImuItem accel_data = accel_buff.interpolateAtTime(update_time);
                    ImuItem gyro_data = gyro_buff.interpolateAtTime(update_time);
                    filter.stepInertial(update_time-last_imu_update_time, accel_data, gyro_data);
                    last_imu_update_time = update_time;
                }
            } else {
                // Propagate camera
                const CameraItem& camera_item = camera_feed.top();
                camera_feed.next();

                std::cout.precision(5);
                std::cout << "Propagate CAM at time " << std::fixed << camera_item.getTime() << std::endl;

                cv::Mat img = camera_feed.getImage(camera_item);

                feature_tracker.processImage(img);

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                pub.publish(msg);
            }
            geometry_msgs::TransformStamped transform;
            Eigen::Quaterniond attitude = filter.getCurrentAttitude();
            Eigen::Vector3d position = filter.getCurrentPosition();
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = "world";
            transform.child_frame_id = "filter";
            transform.transform.translation.x = position(0, 0) / 100;
            transform.transform.translation.y = position(1, 0) / 100;
            transform.transform.translation.z = position(2, 0) / 100;
            transform.transform.rotation.x = attitude.x();
            transform.transform.rotation.y = attitude.y();
            transform.transform.rotation.z = attitude.z();
            transform.transform.rotation.w = attitude.w();
            br.sendTransform(transform);
        }


    } catch (const BaseException& e) {
        std::cerr << "EXCEPTION: " << e.what() << std::endl;
#ifndef NDEBUG
        std::cerr << typeid(e).name() << std::endl;
#endif
        return 1;
    }

    return 0;
}

void Navigator::parseOptions(int argc, const char **argv) {
    namespace po = boost::program_options;

    options_description_.add_options()
            ("help,h", "produce help message")
            ("calib", po::value<std::string>()->default_value("calibration.yaml"), "calibration file")
            ("dataset", po::value<std::string>()->default_value("dataset"), "path to dataset")
            ;

    po::positional_options_description positionalOptions;
    positionalOptions.add("dataset", 1);

    po::store(po::command_line_parser(argc, argv).options(options_description_).positional(positionalOptions).run(), vmap_);
    po::notify(vmap_);
}

void Navigator::printHelp(bool to_stderr) {
    std::ostream& out = to_stderr ? std::cerr : std::cout;
    out << options_description_ << std::endl;
}


