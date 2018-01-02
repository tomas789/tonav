#include "helpers/kitti_loader_helper.h"

#include <sstream>
#include <forward_list>

#include "imu/kitti_imu.h"
#include "vision/kitti_vision.h"
#include "trajectory/kitti_trajectory.h"
#include "sim_setup.h"

namespace Kitti {
    OxtsRecord OxtsRecord::parse(const std::string& line) {
        OxtsRecord r;
        std::sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %d",
                    &r.lat, &r.lon, &r.alt, &r.roll, &r.pitch, &r.yaw, &r.vn, &r.ve, &r.vf, &r.vl, &r.vu, &r.ax, &r.ay,
                    &r.az, &r.af, &r.al, &r.au, &r.wx, &r.wy, &r.wz, &r.wf, &r.wl, &r.wu, &r.pos_accuracy, &r.vel_accuracy,
                    &r.navstat, &r.numsats, &r.posmode, &r.velmode, &r.orimode);
        return r;
    }
}

std::unique_ptr<KittiLoaderHelper> KittiLoaderHelper::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<KittiLoaderHelper> kitti_loader(new KittiLoaderHelper(sim_setup));
    
    sim_setup->setImu(KittiImu::load(sim_setup, *kitti_loader));
    sim_setup->setTrajectory(KittiTrajectory::load(sim_setup, *kitti_loader));
    sim_setup->setVision(KittiVision::load(sim_setup, *kitti_loader));
    
    kitti_loader->dataset_path_ = j["path"];
    kitti_loader->skip_frames_ = j["skip_frames"];
    
    return kitti_loader;
}

void KittiLoaderHelper::initialize(VioSimulation *simulation) {
    std::size_t skip_frames = skip_frames_;
    
    std::string oxts_timestamp_file_name = getOxtsTimestampFileName();
    std::vector<double> oxts_timestamps = loadKittiTimestamps(oxts_timestamp_file_name);
    
    std::string oxts_data_dir = getOxtsDataDirName();
    std::vector<Kitti::OxtsRecord> oxts_records = loadOxtsData(oxts_data_dir, oxts_timestamps.size());
    
    Eigen::Vector2d image_size;
    Eigen::Vector2d focal_length;
    Eigen::Vector2d optical_center;
    std::string cam_to_cam_calibration_file_name = getCamToCamCalibFileName();
    loadCameraParams(cam_to_cam_calibration_file_name, image_size, focal_length, optical_center);
    
    KittiImu& kitti_imu = dynamic_cast<KittiImu&>(sim_setup_->getImu());
    kitti_imu.timestamps_ = std::forward_list<double>(std::begin(oxts_timestamps) + skip_frames, std::end(oxts_timestamps));
    std::map<double, Eigen::Vector3d> gyroscope;
    std::map<double, Eigen::Vector3d> accelerometer;
    for (int i = 0; i < oxts_timestamps.size(); ++i) {
        double timestamp = oxts_timestamps[i];
        const Kitti::OxtsRecord& record = oxts_records[i];
        Eigen::Vector3d accel;
        accel << record.ax, record.ay, record.az;
        Eigen::Vector3d gyro;
        gyro << record.wx, record.wy, record.wz;
        accelerometer.emplace(timestamp, accel);
        gyroscope.emplace(timestamp, gyro);
    }
    kitti_imu.accelerometer_ = accelerometer;
    kitti_imu.gyroscope_ = gyroscope;
    
    KittiVision& kitti_vision = dynamic_cast<KittiVision&>(sim_setup_->getVision());
    kitti_vision.timestamps_ = std::forward_list<double>(std::begin(oxts_timestamps) + skip_frames, std::end(oxts_timestamps));
    kitti_vision.focal_length_ = focal_length;
    kitti_vision.optical_center_ = optical_center;
    std::map<double, std::string> camera_images;
    for (int i = 0; i < oxts_timestamps.size(); ++i) {
        double timestamp = oxts_timestamps[i];
        camera_images.emplace(timestamp, getCameraImageFileName(i));
    }
    kitti_vision.camera_images_ = camera_images;
    
    KittiTrajectory& kitti_trajectory = dynamic_cast<KittiTrajectory&>(sim_setup_->getTrajectory());
    std::string velo_to_cam_file_name = getVeloToCamCalibFileName();
    std::string imu_to_velo_file_name = getImuToVeloCalibFileName();
    std::pair<tonav::Quaternion, Eigen::Vector3d> T_B_C = loadBodyToCameraTransform(velo_to_cam_file_name, imu_to_velo_file_name);
    tonav::Quaternion q_B_C = T_B_C.first;
    Eigen::Vector3d p_B_C = T_B_C.second;
    kitti_trajectory.q_C_B_ = q_B_C.conjugate();
    kitti_trajectory.p_C_B_ = tonav::Geometry::switchFrames(p_B_C, q_B_C);
    
    std::map<double, tonav::Quaternion> q_B_G_gt_;
    std::map<double, Eigen::Vector3d> p_B_G_gt_;
    for (int i = 0; i < oxts_timestamps.size(); ++i) {
        double timestamp = oxts_timestamps[i];
        const Kitti::OxtsRecord& record_start = oxts_records[0];
        const Kitti::OxtsRecord& record = oxts_records[i];
        
        q_B_G_gt_.emplace(timestamp, getRotationFromOxtsRecord(record, record_start));
        p_B_G_gt_.emplace(timestamp, getPositionFromOxtsRecord(record, record_start));
    }
    kitti_trajectory.q_B_G_gt_ = q_B_G_gt_;
    kitti_trajectory.p_B_G_gt_ = p_B_G_gt_;
}

KittiLoaderHelper::KittiLoaderHelper(SimSetup *sim_setup) : Helper(sim_setup) { }
KittiLoaderHelper::~KittiLoaderHelper() = default;

std::vector<double> KittiLoaderHelper::loadKittiTimestamps(const std::string file_name) const {
    std::ifstream file(file_name);
    if (!file) {
        throw std::runtime_error("Cannot open file '" + file_name + "'");
    }
    
    using clock_type = std::chrono::system_clock;
    using time_point_type = std::chrono::time_point<clock_type>;
    
    std::string line;
    time_point_type first_time_point;
    
    std::vector<double> timestamps;
    for (int i = 0; std::getline(file, line); ++i) {
        time_point_type time_point = parseKittiDateTime(line);
        if (i == 0) {
            first_time_point = time_point;
        }
        
        auto time_diff = time_point - first_time_point;
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time_diff);
        double timestamp = double(duration.count()) / 1e6;
        timestamps.push_back(timestamp);
    }
    
    return timestamps;
}

std::vector<Kitti::OxtsRecord> KittiLoaderHelper::loadOxtsData(const std::string& oxts_data_dir, int number_of_records) const {
    std::vector<Kitti::OxtsRecord> records;
    for (std::size_t i = 0; i < number_of_records; ++i) {
        std::ostringstream file_name_stream;
        file_name_stream << oxts_data_dir << "/" << std::setfill('0') << std::setw(10) << i << ".txt";
        std::ifstream file(file_name_stream.str());
        if (!file) {
            throw std::runtime_error("Cannot open file '" + file_name_stream.str() + "'");
        }
        std::string line;
        std::getline(file, line);
        records.push_back(Kitti::OxtsRecord::parse(line));
    }
    return records;
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> KittiLoaderHelper::parseCalibrationFile(const std::string& file_name) const {
    std::ifstream file(file_name);
    if (!file) {
        throw std::runtime_error("Cannot open file '" + file_name + "'");
    }
    std::string calib_time_line;
    std::getline(file, calib_time_line);
    std::string r_line;
    std::getline(file, r_line);
    std::string t_line;
    std::getline(file, t_line);
    
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    std::string r_line_mat_part = r_line.substr(2, std::string::npos);
    std::sscanf(r_line_mat_part.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &r11, &r12, &r13, &r21, &r22, &r23, &r31, &r32, &r33);
    Eigen::Matrix3d r;
    r << r11, r12, r13, r21, r22, r23, r31, r32, r33;
    
    double t1, t2, t3;
    std::string t_line_mat_part = t_line.substr(2, std::string::npos);
    std::sscanf(t_line_mat_part.c_str(), "%lf %lf %lf", &t1, &t2, &t3);
    Eigen::Vector3d t;
    t << t1, t2, t3;
    
    return std::make_pair(r, t);
}

std::pair<tonav::Quaternion, Eigen::Vector3d> KittiLoaderHelper::loadBodyToCameraTransform(const std::string& velo_to_cam_file_name, const std::string& imu_to_velo_file_name) const {
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> data_velo_to_cam = parseCalibrationFile(velo_to_cam_file_name);
    Eigen::Matrix3d R_cam_velo = data_velo_to_cam.first;
    Eigen::Vector3d p_velo_cam = data_velo_to_cam.second;
    
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> data_imu_to_velo = parseCalibrationFile(imu_to_velo_file_name);
    Eigen::Matrix3d R_velo_imu = data_imu_to_velo.first;
    Eigen::Vector3d p_imu_velo = data_imu_to_velo.second;
    
    Eigen::Matrix3d R_cam_imu = R_cam_velo * R_velo_imu;
    Eigen::Vector3d p_cam_velo = -1 * R_cam_velo.transpose() * p_velo_cam;
    
    Eigen::Vector3d p_imu_cam = R_cam_velo*(p_imu_velo - p_cam_velo);
    
    tonav::Quaternion q_B_C = tonav::Quaternion::fromRotationMatrix(R_cam_imu.transpose());
    Eigen::Vector3d p_B_C = p_imu_cam;
    return std::make_pair(q_B_C, p_B_C);
}

std::chrono::time_point<std::chrono::system_clock> KittiLoaderHelper::parseKittiDateTime(const std::string& str) const {
    std::tm t = {};
    std::istringstream ss(str);
    ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");
    long long us = std::stoll(str.substr(20, 6));
    using clock_type = std::chrono::system_clock;
    return clock_type::from_time_t(std::mktime(&t)) + std::chrono::microseconds(us);
}

void KittiLoaderHelper::loadCameraParams(const std::string &file_name, Eigen::Vector2d& image_size, Eigen::Vector2d& focal_length, Eigen::Vector2d& optical_center) const {
    std::ifstream cam_to_cam(file_name);
    if (!cam_to_cam) {
        throw std::runtime_error("Cannot open file " + file_name);
    }
    int i = 1;
    bool s_matrix_loaded = false;
    bool p_matrix_loaded = false;
    for (std::string line; std::getline(cam_to_cam, line); ++i) {
        std::vector<std::string> parts;
        std::istringstream line_stream(line);
        for (std::string item; std::getline(line_stream, item, ' '); ) {
            parts.push_back(item);
        }
        if (parts.empty()) {
            continue;
        }
        std::string key = parts[0];
        if (key == "S_rect_00:") {
            if (parts.size() != 3) {
                throw std::runtime_error("Expected 3 items at line " + std::to_string(i) + " of file " + file_name);
                std::exit(1);
            }
            image_size(0) = std::stod(parts[1]);
            image_size(1) = std::stod(parts[2]);
            s_matrix_loaded = true;
        } else if (key == "P_rect_00:") {
            if (parts.size() != 13) {
                throw std::runtime_error("Expected 13 items at line " + std::to_string(i) + " of file " + file_name);
                std::exit(1);
            }
            focal_length(0) = std::stod(parts[1]);
            focal_length(1) = std::stod(parts[6]);
            optical_center(0) = std::stod(parts[3]);
            optical_center(1) = std::stod(parts[7]);
            p_matrix_loaded = true;
        }
    }
    if (!s_matrix_loaded || !p_matrix_loaded) {
        throw std::runtime_error("Loading camera calibration failed.");
    }
}

tonav::Quaternion KittiLoaderHelper::getRotationFromOxtsRecord(const Kitti::OxtsRecord& record, const Kitti::OxtsRecord& record_start) const {
    tonav::Quaternion q_Wi_GPS = getGpsRotationFromOxtsRecord(record);
    tonav::Quaternion q_GPS_W0 = getGpsRotationFromOxtsRecord(record_start).conjugate();
    tonav::Quaternion q_Wi_W0 = q_Wi_GPS*q_GPS_W0;
    return q_Wi_W0;
}

tonav::Quaternion KittiLoaderHelper::getGpsRotationFromOxtsRecord(const Kitti::OxtsRecord& record) const {
    Eigen::Matrix3d rx = rotx(record.roll);
    Eigen::Matrix3d ry = roty(record.pitch);
    Eigen::Matrix3d rz = rotz(record.yaw);
    Eigen::Matrix3d R_GPS_Wi = rz*ry*rx;
    Eigen::Matrix3d R_Wi_GPS = R_GPS_Wi.transpose();
    return tonav::Quaternion::fromRotationMatrix(R_Wi_GPS);
}

Eigen::Vector3d KittiLoaderHelper::getPositionFromOxtsRecord(const Kitti::OxtsRecord& record, const Kitti::OxtsRecord& record_start) const {
    tonav::Quaternion q_W0_GPS = getGpsRotationFromOxtsRecord(record_start);
    Eigen::Matrix3d R_W0_GPS = q_W0_GPS.toRotationMatrix();
    Eigen::Vector3d p_Wi_GPS = getGpsPositionFromOxtsRecord(record, record_start);
    Eigen::Vector3d p_W0_GPS = getGpsPositionFromOxtsRecord(record_start, record_start);
    return R_W0_GPS*(p_Wi_GPS - p_W0_GPS);
}

Eigen::Vector3d KittiLoaderHelper::getGpsPositionFromOxtsRecord(const Kitti::OxtsRecord& record, const Kitti::OxtsRecord& record_start) const {
    double er = 6378137.0;
    double scale = std::cos(record_start.lat * M_PI / 180.0);
    double tx = scale * record.lon * M_PI * er / 180.0;
    double ty = scale * er * std::log(std::tan((90.0 + record.lat) * M_PI / 360.0));
    double tz = record.alt;
    
    Eigen::Vector3d t;
    t << tx, ty, tz;
    return t;
}

Eigen::Matrix3d KittiLoaderHelper::rotx(double t) const {
    double c = std::cos(t);
    double s = std::sin(t);
    Eigen::Matrix3d r;
    r << 1, 0, 0, 0, c, -1*s, 0, s, c;
    return r;
}

Eigen::Matrix3d KittiLoaderHelper::roty(double t) const {
    double c = std::cos(t);
    double s = std::sin(t);
    Eigen::Matrix3d r;
    r << c, 0, s, 0, 1, 0, -1*s, 0, c;
    return r;
}

Eigen::Matrix3d KittiLoaderHelper::rotz(double t) const {
    double c = std::cos(t);
    double s = std::sin(t);
    Eigen::Matrix3d r;
    r << c, -1*s, 0, s, c, 0, 0, 0, 1;
    return r;
}

std::string KittiLoaderHelper::getImuToVeloCalibFileName() const {
    return dataset_path_ + "/../calib_imu_to_velo.txt";
}

std::string KittiLoaderHelper::getVeloToCamCalibFileName() const {
    return dataset_path_ + "/../calib_velo_to_cam.txt";
}

std::string KittiLoaderHelper::getCamToCamCalibFileName() const {
    return dataset_path_ + "/../calib_cam_to_cam.txt";
}

std::string KittiLoaderHelper::getOxtsTimestampFileName() const {
    return dataset_path_ + "/oxts/timestamps.txt";
}

std::string KittiLoaderHelper::getOxtsDataDirName() const {
    return dataset_path_ + "/oxts/data";
}

std::string KittiLoaderHelper::getCameraImageFileName(int i) const {
    std::ostringstream file_name_stream;
    file_name_stream << dataset_path_ + "/image_00/data/" << std::setfill('0') << std::setw(10) << i << ".png";
    return file_name_stream.str();
}
