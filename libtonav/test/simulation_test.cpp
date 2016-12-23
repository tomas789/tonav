#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "BezierCurve.h"
#include "BezierNode.h"

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>


TEST(SimulationTest, test_bezier_path_move_forward) {
    std::vector<Eigen::Vector3f> waypoints; /* Length N */
    waypoints.push_back(0.0*Eigen::Vector3f::UnitX());
    waypoints.push_back(1.0*Eigen::Vector3f::UnitX());
    waypoints.push_back(2.0*Eigen::Vector3f::UnitX());
    waypoints.push_back(3.0*Eigen::Vector3f::UnitX());

    BezierCurve<Eigen::Vector3f> curve(waypoints, 1.0f);

    ASSERT_EQ(curve.positionAtTime(0.0), 0.0*Eigen::Vector3f::UnitX());
    ASSERT_EQ(curve.positionAtTime(1.0), 1.0*Eigen::Vector3f::UnitX());
    ASSERT_EQ(curve.positionAtTime(2.0), 2.0*Eigen::Vector3f::UnitX()) << "Got [" << curve.positionAtTime(2.0).transpose() << "]^T" << std::endl;
    ASSERT_EQ(curve.positionAtTime(3.0), 3.0*Eigen::Vector3f::UnitX()) << "Got [" << curve.positionAtTime(3.0).transpose() << "]^T" << std::endl;
}

TEST(SimulationTest, test_bezier_path_cycle) {
    float r = 5.0;
    Eigen::Vector3f a, b, c, d;
    a << 0, -r, 0;
    b << r, 0, 0;
    c << 0, r, 0;
    d << -r, 0, 0;
    std::vector<Eigen::Vector3f> waypoints; /* Length N */
    for (std::size_t i = 0; i < 20; ++i) {
        switch (i%4) {
            case 0:
                waypoints.push_back(a);
                break;
            case 1:
                waypoints.push_back(b);
                break;
            case 2:
                waypoints.push_back(c);
                break;
            case 3:
                waypoints.push_back(d);
                break;
        }
    }

    BezierCurve<Eigen::Vector3f> curve(waypoints, 1.0f);

    ASSERT_EQ(curve.positionAtTime(0.0), a);
    ASSERT_EQ(curve.positionAtTime(4.0), a) << "Got position [" << curve.positionAtTime(4.0).transpose() << "]^T" << std::endl;
    ASSERT_EQ(curve.positionAtTime(8.0), a);
    ASSERT_EQ(curve.positionAtTime(12.0), a);
    ASSERT_EQ(curve.positionAtTime(16.0), a);
}

TEST(SimulationTest, test_ros_simple_publish) {
    float r = 1.0;
    std::vector<Eigen::Vector3f> waypoints; /* Length N */
    std::vector<Eigen::Quaternionf> waypoints_q; /* Length N */
    int nodes = 60;
    for (std::size_t j = 0; j <= nodes; ++j) {
        Eigen::Vector3f wp;
        float t = j*2*M_PI/nodes;
        wp << r*std::sin(t), r*std::cos(t), 0;
        waypoints.push_back(wp);

        double theta = t;
        Eigen::Quaternionf q(std::cos(theta/2), 0, 0, std::sin(theta/2));
        waypoints_q.push_back(q);
    }
    BezierCurve<Eigen::Vector3f> curve(waypoints, 1.0/nodes);
    BezierCurve<Eigen::Quaternionf> curve_q(waypoints_q, 1.0/nodes);

    std::vector<Eigen::Vector3f> features;
    float size = 6.0;
    for (std::size_t i = 0; i < 5; ++i) {
        // Randomly chosen prime numbers
        int shift = 1.0;
        float a = fmod(fmod((i+shift)*11155127.0, 27611.0), size) - size/2.0;
        float b = fmod(fmod((i+shift)*13664839.0, 2128319.0), size) - size/2.0;
        Eigen::Vector3f p;
        p << size/2.0, a, b;
        features.push_back(p);
    }
    for (std::size_t i = 0; i < 5; ++i) {
        // Randomly chosen prime numbers
        int shift = 2.0;
        float a = fmod(fmod((i+shift)*11155127.0, 27611.0), size) - size/2.0;
        float b = fmod(fmod((i+shift)*13664839.0, 2128319.0), size) - size/2.0;
        Eigen::Vector3f p;
        p << -size/2.0, a, b;
        features.push_back(p);
    }
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
    cloud.points.resize(features.size());
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(features.size());
    for (std::size_t i = 0; i < features.size(); ++i) {
        cloud.points[i].x = features[i](0);
        cloud.points[i].y = features[i](1);
        cloud.points[i].z = features[i](2);
        cloud.channels[0].values[i] = 1;
    }

    int argc = 1;
    char* argv[] = { (char*)"SimulationTest_test_ros_simple_publish" };
    ros::init(argc, argv, "SimulationTest_test_ros_simple_publish");
    ros::NodeHandle private_node("~");
    tf2_ros::TransformBroadcaster br;
    ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud>("/pointcloud", 1);
    ros::Rate rate(100);
    for (std::size_t cycle = 0; cycle < 5; ++cycle) {
        for (std::size_t i = 0; i < 500; ++i) {
            float t = i/500.0;
            Eigen::Vector3f p = curve.positionAtTime(t);
            Eigen::Quaternionf q = curve_q.quaternionAtTime(t).conjugate();
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "Test";
            transformStamped.transform.translation.x = p(0);
            transformStamped.transform.translation.y = p(1);
            transformStamped.transform.translation.z = p(2);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            br.sendTransform(transformStamped);
            rate.sleep();
        }
    }
}