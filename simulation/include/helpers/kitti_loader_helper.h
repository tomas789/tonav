//
// Created by Tomas Krejci on 26/12/17.
//

#ifndef TONAV_KITTI_LOADER_HELPER_H
#define TONAV_KITTI_LOADER_HELPER_H

#include <json.hpp>

#include "../helper.h"

class VioSimulation;

using json = nlohmann::json;

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

class KittiLoaderHelper: public Helper {
public:
    static std::unique_ptr<KittiLoaderHelper> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    static inline std::string getComponentName() {
        return "kitti_loader";
    }
    
    virtual ~KittiLoaderHelper();
    
protected:
    KittiLoaderHelper(SimSetup *sim_setup);
    
    std::vector<double> loadKittiTimestamps(const std::string file_name) const;
    std::vector<Kitti::OxtsRecord> loadOxtsData(const std::string& oxts_data_dir, int number_of_records) const;
    
    std::chrono::time_point<std::chrono::system_clock> parseKittiDateTime(const std::string& str) const;
    
    void loadCameraParams(const std::string &file_name, Eigen::Vector2d& image_size, Eigen::Vector2d& focal_length, Eigen::Vector2d& optical_center) const;
    
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> parseCalibrationFile(const std::string& filename) const;
    std::pair<tonav::Quaternion, Eigen::Vector3d> loadBodyToCameraTransform(const std::string& velo_to_cam_file_name, const std::string& imu_to_velo_file_name) const;
    
    
    /// p_Wi_W0 == p_Bi_G
    Eigen::Vector3d getPositionFromOxtsRecord(const Kitti::OxtsRecord& record, const Kitti::OxtsRecord& record_start) const;
    
    /// q_Wi_W0 == q_Bi_G
    tonav::Quaternion getRotationFromOxtsRecord(const Kitti::OxtsRecord& record, const Kitti::OxtsRecord& record_start) const;
    
    /// q_Wi_GPS
    tonav::Quaternion getGpsRotationFromOxtsRecord(const Kitti::OxtsRecord& record) const;
    
    /// p_Wi_GPS
    Eigen::Vector3d getGpsPositionFromOxtsRecord(const Kitti::OxtsRecord& record, const Kitti::OxtsRecord& record_start) const;
    
    Eigen::Matrix3d rotx(double t) const;
    Eigen::Matrix3d roty(double t) const;
    Eigen::Matrix3d rotz(double t) const;
    
    std::string getImuToVeloCalibFileName() const;
    std::string getVeloToCamCalibFileName() const;
    std::string getCamToCamCalibFileName() const;
    std::string getOxtsTimestampFileName() const;
    std::string getOxtsDataDirName() const;
    std::string getCameraImageFileName(int i) const;
    
    std::string dataset_path_;
    std::size_t skip_frames_;
};

#endif //TONAV_KITTI_LOADER_HELPER_H
