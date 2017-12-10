#ifndef TONAV_TONAV_KITTI_H
#define TONAV_TONAV_KITTI_H

#include <chrono>
#include <memory>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>

#include "calibration.h"
#include "tonav.h"

namespace Kitti {
    struct OxtsRecord {
        double lat;   // latitude of the oxts-unit (deg)
        double lon;   // longitude of the oxts-unit (deg)
        double alt;   // altitude of the oxts-unit (m)
        double roll;  // roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
        double pitch; // pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
        double yaw;   // heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
        double vn;    // velocity towards north (m/s)
        double ve;    // velocity towards east (m/s)
        double vf;    // forward velocity, i.e. parallel to earth-surface (m/s)
        double vl;    // leftward velocity, i.e. parallel to earth-surface (m/s)
        double vu;    // upward velocity, i.e. perpendicular to earth-surface (m/s)
        double ax;    // acceleration in x, i.e. in direction of vehicle front (m/s^2)
        double ay;    // acceleration in y, i.e. in direction of vehicle left (m/s^2)
        double az;    // acceleration in z, i.e. in direction of vehicle top (m/s^2)
        double af;    // forward acceleration (m/s^2)
        double al;    // leftward acceleration (m/s^2)
        double au;    // upward acceleration (m/s^2)
        double wx;    // angular rate around x (rad/s)
        double wy;    // angular rate around y (rad/s)
        double wz;    // angular rate around z (rad/s)
        double wf;    // angular rate around forward axis (rad/s)
        double wl;    // angular rate around leftward axis (rad/s)
        double wu;    // angular rate around upward axis (rad/s)
        double pos_accuracy;  // velocity accuracy (north/east in m)
        double vel_accuracy;  // velocity accuracy (north/east in m/s)
        int navstat;       // navigation status (see navstat_to_string)
        int numsats;       // number of satellites tracked by primary GPS receiver
        int posmode;       // position mode of primary GPS receiver (see gps_mode_to_string)
        int velmode;       // velocity mode of primary GPS receiver (see gps_mode_to_string)
        int orimode;       // orientation mode of primary GPS receiver (see gps_mode_to_string)
        
        static OxtsRecord parse(const std::string& line);
    };
}

class TonavKitti {
public:
    TonavKitti();
    
    int run(int argc, char* argv[]);
private:
    using clock_type = std::chrono::system_clock;
    using time_point_type = std::chrono::time_point<clock_type>;
    
    double factor_ = 25;
    
    std::string calibration_file_;
    
    std::string kitti_dir_;
    std::string kitti_date_;
    std::string kitti_drive_;
    
    std::vector<time_point_type> oxts_timestamps_;
    std::vector<Kitti::OxtsRecord> oxts_;
    
    Eigen::Matrix3d R_B_C_;
    Eigen::Vector3d p_B_C_;
    
    Eigen::Vector2d image_size_;
    Eigen::Vector2d focal_length_;
    Eigen::Vector2d optical_center_;
    
    std::unique_ptr<Tonav> tonav_;
    std::shared_ptr<Calibration> calibration_;
    time_point_type initialization_time_;
    
    std::string getImuToVeloCalibFileName() const;
    std::string getVeloToCamCalibFileName() const;
    std::string getCamToCamCalibFileName() const;
    std::string getOxtsTimestampFileName() const;
    std::string getOxtsDataDirName() const;
    std::string getCameraImageFileName(int i) const;
    
    time_point_type parseKittiDateTime(const std::string& str) const;
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> parseCalibrationFile(const std::string& filename) const;
    
    void loadOxtsTimestamps();
    void loadOxtsData();
    void loadCalibration();
    
    void initialize();
    cv::Mat step(std::size_t i);
    
    Eigen::Vector3d getGroundTruthHelper(std::size_t i) const;
    Eigen::Vector3d getGroundTruthPosition(std::size_t i) const;
    Eigen::Matrix3d getGroundTruthRotation(std::size_t i) const;
    
    std::vector<Eigen::Vector2d> bodyFrameMarker() const;

    void dumpLogs(std::ofstream& out, const Quaternion& q_B_G, const Quaternion& q_B_G_true, const Eigen::Vector3d& p_B_G, const Eigen::Vector3d& p_B_G_true, const Eigen::Vector3d& v_B_G, const Eigen::Vector3d& v_B_G_true);

    /**
     * @brief Publish localization results to ROS TF
     *
     * TF expects quaternion in Hamilton notation that rotates vectors and is right-handed (!). In Tonav, I use JPL
     * notation that transforms frames and is left-handed. Those two factors cancel out so I should pass directly my
     * quaternions to ROS.
     */
    void publishTransformations(std::size_t i, tf2_ros::TransformBroadcaster& broadcaster);
    void publishPointCloud(ros::Publisher& publisher);
    
    /// This function converts decimal degrees to radians
    double deg2rad(double deg) const;
    
    /**
     * Returns the distance between two points on the Earth.
     * Direct translation from http://en.wikipedia.org/wiki/Haversine_formula
     * @param lat1d Latitude of the first point in degrees
     * @param lon1d Longitude of the first point in degrees
     * @param lat2d Latitude of the second point in degrees
     * @param lon2d Longitude of the second point in degrees
     * @return The distance between the two points in meters
     */
    double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) const;
    
    constexpr static double earthRadiusM = 6371000.0;
    
    Eigen::Matrix3d rotx(double t) const;
    Eigen::Matrix3d roty(double t) const;
    Eigen::Matrix3d rotz(double t) const;
};

std::ostream& operator<<(std::ostream& out, const Kitti::OxtsRecord& r);

#endif //TONAV_TONAV_KITTI_H
