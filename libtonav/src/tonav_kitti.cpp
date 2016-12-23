#include "tonav_kitti.h"

#include <iostream>
#include <cstdio>
#include <iomanip>
#include <fstream>
#include <utility>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>

#include "quaternion.h"

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

TonavKitti::TonavKitti() {
    
}

int TonavKitti::run(int argc, char *argv[]) {
    std::cout << "Tonav KITTI - tomas789@gmail.com" << std::endl;
    
    if (argc != 5) {
        std::cerr << "usage: tonavkitti kitti_base_dir date drive calibration_file" << std::endl;
        return 1;
    }
    
    kitti_dir_ = std::string(argv[1]);
    kitti_date_ = std::string(argv[2]);
    kitti_drive_ = std::string(argv[3]);
    calibration_file_ = std::string(argv[4]);
    
    ros::init(argc, argv, "tonavkitti");
    ros::NodeHandle nh;
    ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud>("/tonav/pointcloud", 1);
    tf2_ros::TransformBroadcaster broadcaster;
    
    loadOxtsTimestamps();
    loadOxtsData();
    loadCalibration();
    
    cv::namedWindow("Tonav", cv::WINDOW_AUTOSIZE);
    
    std::cout << " ⛳ TONAVKITTI INITIALIZE ⛳ " << std::endl;
    initialize();
    std::cout << " ⛳ DONE - START LOCALIZATION ⛳ " << std::endl;
    
    std::ofstream gt_pos(kitti_dir_ + "/../gt.csv");
    std::ofstream feature_pc(kitti_dir_ + "/../feature_pc.csv");
    std::ofstream marker_pts(kitti_dir_ + "/../marker_pts.csv");
    std::ofstream logger(kitti_dir_ + "/../tonavkitti.log");
    
    bool is_initialized = false;
    std::size_t max_steps = oxts_timestamps_.size();
    max_steps = 239;
    for (std::size_t i = 0; i < max_steps && ros::ok(); ++i) {
        std::cout << " ⛳ STEP " << i << " ⛳ " << std::endl;
        cv::Mat frame = step(i);
        
        if (is_initialized) {
            cv::Mat image = tonav_->getCurrentImage();
            std::cout << "image " << image.cols << "x" << image.rows << std::endl;
            cv::imshow("Tonav", frame);
            cv::waitKey(1);
            
            Eigen::Vector3d position = tonav_->getCurrentPosition();
            Eigen::Vector3d true_position = getGroundTruthPosition(i-1);
            
            std::cout << "POSITION: [" << position.transpose() << "]^T" << std::endl;
            std::cout << "GROUND TRUTH: [" << true_position.transpose() << "]^T" << std::endl;
            std::cout << "ERROR: [" << (position - true_position).transpose() << "]^T" << std::endl;
            
            gt_pos << position(0) << "," << position(1) << "," << position(2) << "," << true_position(0) << "," << true_position(1) << "," << true_position(2) << std::endl;
            
            std::vector<Eigen::Vector3d> feature_pointcloud = tonav_->featurePointCloud();
            std::cout << "Got " << feature_pointcloud.size() << " features." << std::endl;
            for (const Eigen::Vector3d& feature : feature_pointcloud) {
                feature_pc << i << "," << feature(0) << "," << feature(1) << "," << feature(2) << std::endl;
            }
            
            std::vector<Eigen::Vector2d> marker = bodyFrameMarker();
            assert(marker.size() == 3);
            marker_pts
                << marker[0](0) << "," << marker[0](1) << ","
                << marker[1](0) << "," << marker[1](1) << ","
                << marker[2](0) << "," << marker[2](1) << std::endl;
            
            publishTransformations(i, broadcaster);
            publishPointCloud(pointcloud_publisher);


            Eigen::Matrix3d R_G_W = getGroundTruthRotation(0).transpose();
            Eigen::Matrix3d R_Bi_G = getGroundTruthRotation(i-1);

            const Quaternion& q_B_G = tonav_->getCurrentOrientation();
            const Quaternion& q_B_G_true = Quaternion::fromRotationMatrix(R_Bi_G*R_G_W).conjugate();
            const Eigen::Vector3d& p_B_G = tonav_->getCurrentPosition();
            const Eigen::Vector3d& p_B_G_true = getGroundTruthPosition(i-1);
            const Eigen::Vector3d& v_B_G = tonav_->getCurrentVelocity();
            const Eigen::Vector3d& v_B_G_true = Eigen::Vector3d::Zero();

            dumpLogs(logger, q_B_G, q_B_G_true, p_B_G, p_B_G_true, v_B_G, v_B_G_true);
        }
        
        time_point_type current_time = clock_type::now();
        double runtime = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(current_time-initialization_time_).count())/1e6;
        std::cout << " ⛸ DONE TIME " << tonav_->time() << " | RUNTIME " << runtime << " | REAL TIME FACTOR " << (tonav_->time() / runtime) << std::endl;
        
        is_initialized = tonav_->isInitialized();
    }
    
    return 0;
}

std::string TonavKitti::getImuToVeloCalibFileName() const {
    return kitti_dir_ + "/" + kitti_date_ + "/calib_imu_to_velo.txt";
}

std::string TonavKitti::getVeloToCamCalibFileName() const {
    return kitti_dir_ + "/" + kitti_date_ + "/calib_velo_to_cam.txt";
}

std::string TonavKitti::getCamToCamCalibFileName() const {
    return kitti_dir_ + "/" + kitti_date_ + "/calib_cam_to_cam.txt";
}

std::string TonavKitti::getOxtsTimestampFileName() const {
    return kitti_dir_ + "/" + kitti_date_ + "/" + kitti_date_ + "_drive_" + kitti_drive_ + "_sync/oxts/timestamps.txt";
}

std::string TonavKitti::getOxtsDataDirName() const {
    return kitti_dir_ + "/" + kitti_date_ + "/" + kitti_date_ + "_drive_" + kitti_drive_ + "_sync/oxts/data";
}

std::string TonavKitti::getCameraImageFileName(int i) const {
    std::ostringstream file_name_stream;
    file_name_stream << kitti_dir_ << "/" << kitti_date_ << "/" << kitti_date_ << "_drive_" << kitti_drive_ << "_sync/image_00/data/" << std::setfill('0') << std::setw(10) << i << ".png";
    return file_name_stream.str();
}

TonavKitti::time_point_type TonavKitti::parseKittiDateTime(const std::string& str) const {
    std::tm t = {};
    std::istringstream ss(str);
    ss >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");
    long long us = std::stoll(str.substr(20, 6));
    return clock_type::from_time_t(std::mktime(&t)) + std::chrono::microseconds(us);
}

std::pair<Eigen::Matrix3d, Eigen::Vector3d> TonavKitti::parseCalibrationFile(const std::string& filename) const {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Cannot open file '" << filename << "'" << std::endl;
        std::exit(1);
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

void TonavKitti::loadOxtsTimestamps() {
    std::string filename = getOxtsTimestampFileName();
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Cannot open file '" << filename << "'" << std::endl;
        std::exit(1);
    }
    for (std::string line; std::getline(file, line); ) {
        oxts_timestamps_.push_back(parseKittiDateTime(line));
    }
    std::cout << "Loaded " << oxts_timestamps_.size() << " timestamps." << std::endl;
}

void TonavKitti::loadOxtsData() {
    std::string data_dir = getOxtsDataDirName();

    for (std::size_t i = 0; i < oxts_timestamps_.size(); ++i) {
        std::ostringstream file_name_stream;
        file_name_stream << data_dir << "/" << std::setfill('0') << std::setw(10) << i << ".txt";
        std::ifstream file(file_name_stream.str());
        if (!file) {
            std::cerr << "Cannot open file '" << file_name_stream.str() << "'" << std::endl;
            std::exit(1);
        }
        std::string line;
        std::getline(file, line);
        oxts_.push_back(Kitti::OxtsRecord::parse(line));
    }
}

void TonavKitti::loadCalibration() {
    std::string velo_to_cam_calibration_filename = getVeloToCamCalibFileName();
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> data_velo_to_cam = parseCalibrationFile(velo_to_cam_calibration_filename);
    Eigen::Matrix3d R_cam_velo = data_velo_to_cam.first;
    Eigen::Vector3d p_velo_cam = data_velo_to_cam.second;
    
    std::string imu_to_velo_calibration_filename = getImuToVeloCalibFileName();
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> data_imu_to_velo = parseCalibrationFile(imu_to_velo_calibration_filename);
    Eigen::Matrix3d R_velo_imu = data_imu_to_velo.first;
    Eigen::Vector3d p_imu_velo = data_imu_to_velo.second;
    
    Eigen::Matrix3d R_cam_imu = R_cam_velo * R_velo_imu;
    Eigen::Vector3d p_cam_velo = -1 * R_cam_velo.transpose() * p_velo_cam;
    
    Eigen::Vector3d p_imu_cam = R_cam_velo*(p_imu_velo - p_cam_velo);
    
    R_B_C_ = R_cam_imu.transpose();
    p_B_C_ = p_imu_cam;
    
    std::string cam_to_cam_calibration_filename = getCamToCamCalibFileName();
    std::ifstream cam_to_cam(cam_to_cam_calibration_filename);
    if (!cam_to_cam) {
        throw std::runtime_error("Cannot open file " + cam_to_cam_calibration_filename);
        std::exit(1);
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
                throw std::runtime_error("Expected 3 items at line " + std::to_string(i) + " of file " + cam_to_cam_calibration_filename);
                std::exit(1);
            }
            image_size_(0) = std::stod(parts[1]);
            image_size_(1) = std::stod(parts[2]);
            s_matrix_loaded = true;
        } else if (key == "P_rect_00:") {
            if (parts.size() != 13) {
                throw std::runtime_error("Expected 13 items at line " + std::to_string(i) + " of file " + cam_to_cam_calibration_filename);
                std::exit(1);
            }
            focal_length_(0) = std::stod(parts[1]);
            focal_length_(1) = std::stod(parts[6]);
            optical_center_(0) = std::stod(parts[3]);
            optical_center_(1) = std::stod(parts[7]);
            p_matrix_loaded = true;
        }
    }
    if (!s_matrix_loaded || !p_matrix_loaded) {
        throw std::runtime_error("Loading camera calibration failed.");
        std::exit(1);
    }
}

void TonavKitti::initialize() {
    initialization_time_ = clock_type::now();
    calibration_ = Calibration::fromPath(calibration_file_);
    calibration_->setCameraOpticalCenter(optical_center_);
    calibration_->setCameraFocalPoint(focal_length_);
    calibration_->setCameraRadialDistortionParams(Eigen::Vector3d::Zero());
    calibration_->setCameraTangentialDistortionParams(Eigen::Vector2d::Zero());
    calibration_->setCameraDelayTime(0.0);
    calibration_->setCameraReadoutTime(0.0);
    calibration_->setBodyToCameraRotation(Quaternion::fromRotationMatrix(R_B_C_.transpose()));
    tonav_.reset(new Tonav(calibration_, p_B_C_));
    Eigen::Vector3d velocity;
    velocity << oxts_[0].vf, oxts_[0].vl, oxts_[0].vu;
    tonav_->initializer()->setVelocity(velocity);
}

cv::Mat TonavKitti::step(std::size_t i) {
    double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(oxts_timestamps_[i] - oxts_timestamps_[0]).count())/1e6;
    Eigen::Vector3d accel;
    accel << oxts_[i].af, oxts_[i].al, oxts_[i].au;
    Eigen::Vector3d gyro;
    gyro << oxts_[i].wf, oxts_[i].wl, oxts_[i].wu;
    
    tonav_->updateAcceleration(time, accel);
    tonav_->updateRotationRate(time, gyro);
    
    cv::Mat previous_frame = tonav_->isInitialized() ? tonav_->getCurrentImage() : cv::Mat();

    if (tonav_->isInitialized()) {
        bool use_orientation_ground_truth = false;
        bool use_position_ground_truth = false;
        bool use_velocity_ground_truth = false;

        if (use_orientation_ground_truth) {
            Quaternion gt_orientation = Quaternion::fromRotationMatrix(
                    getGroundTruthRotation(i-1)*getGroundTruthRotation(0).transpose());
            tonav_->orientationCorrection(gt_orientation.conjugate());
        }

        if (use_position_ground_truth) {
            tonav_->positionCorrection(getGroundTruthPosition(i-1));
        }

        if (use_velocity_ground_truth) {
            Eigen::Vector3d gt_velocity;
            gt_velocity << oxts_[i-1].vf, oxts_[i-1].vl, oxts_[i-1].vu;
            tonav_->velocityCorrection(gt_velocity);
        }
    }
    
    cv::Mat image = cv::imread(getCameraImageFileName(i));
    tonav_->updateImage(time, image);
    
    return previous_frame;
}

Eigen::Vector3d TonavKitti::getGroundTruthHelper(std::size_t i) const {
    double er = 6378137.0;
    double scale = std::cos(oxts_[0].lat * M_PI / 180.0);
    double tx = scale * oxts_[i].lon * M_PI * er / 180.0;
    double ty = scale * er * std::log(std::tan((90.0 + oxts_[i].lat) * M_PI / 360.0));
    double tz = oxts_[i].alt;
    
    Eigen::Vector3d t;
    t << tx, ty, tz;
    return t;
}

Eigen::Vector3d TonavKitti::getGroundTruthPosition(std::size_t i) const {
    return getGroundTruthRotation(0).transpose()*(getGroundTruthHelper(i) - getGroundTruthHelper(0));
}

Eigen::Matrix3d TonavKitti::getGroundTruthRotation(std::size_t i) const {
    Eigen::Matrix3d rx = rotx(oxts_[i].roll);
    Eigen::Matrix3d ry = roty(oxts_[i].pitch);
    Eigen::Matrix3d rz = rotz(oxts_[i].yaw);
    return rz*ry*rx;
}

std::vector<Eigen::Vector2d> TonavKitti::bodyFrameMarker() const {
    double a = 1.0;
    Eigen::Vector3d p_x0_B;
    p_x0_B << 0.0, 0.0, 0.0;
    Eigen::Vector3d p_x1_B;
    p_x1_B << std::sqrt(0.75)*a, 0.5*a, 0.0;
    Eigen::Vector3d p_x2_B;
    p_x2_B << std::sqrt(0.75)*a, -0.5*a, 0.5;
    
    Eigen::Matrix3d R_G_B = tonav_->getCurrentOrientation().toRotationMatrix().transpose();
    Eigen::Vector3d p_B_G = tonav_->getCurrentPosition();
    Eigen::Matrix<double, 2, 3> P = Eigen::Matrix<double, 2, 3>::Identity();
    
    return { P*(p_B_G + R_G_B*p_x0_B), P*(p_B_G + R_G_B*p_x1_B), P*(p_B_G + R_G_B*p_x2_B) };
}

void TonavKitti::dumpLogs(std::ofstream& out, const Quaternion& q_B_G, const Quaternion& q_B_G_true, const Eigen::Vector3d& p_B_G, const Eigen::Vector3d& p_B_G_true, const Eigen::Vector3d& v_B_G, const Eigen::Vector3d& v_B_G_true) {
    Quaternion q_error = q_B_G * q_B_G_true.conjugate();
    double angular_error = 2*std::acos(q_error.w());
    double position_error = (p_B_G - p_B_G_true).norm();
    double velocity_error = (v_B_G - v_B_G_true).norm();
    out << angular_error << "," << position_error << "," << velocity_error << ","
    << p_B_G(0) << "," << p_B_G(1) << "," << p_B_G(2) << ","
    << p_B_G_true(0) << "," << p_B_G_true(1) << "," << p_B_G_true(2) << std::endl;
    out.flush();
}

void TonavKitti::publishTransformations(std::size_t i, tf2_ros::TransformBroadcaster& broadcaster) {
    {
        Eigen::Vector3d body_position = tonav_->getCurrentPosition() / factor_;
        Quaternion body_orientation = tonav_->getCurrentOrientation();
        
        geometry_msgs::TransformStamped body;
        body.header.stamp = ros::Time::now();
        body.header.frame_id = "world";
        body.child_frame_id = "tonavkitti";
        body.transform.translation.x = body_position(0);
        body.transform.translation.y = body_position(1);
        body.transform.translation.z = body_position(2);
        body.transform.rotation.x = body_orientation.x();
        body.transform.rotation.y = body_orientation.y();
        body.transform.rotation.z = body_orientation.z();
        body.transform.rotation.w = body_orientation.w();
        broadcaster.sendTransform(body);
    }

    {
        Eigen::Vector3d body_position = getGroundTruthPosition(i-1) / factor_;

        Eigen::Matrix3d R_G_W = getGroundTruthRotation(0).transpose();
        Eigen::Matrix3d R_Bi_G = getGroundTruthRotation(i-1);
        Quaternion gt_orientation = Quaternion::fromRotationMatrix(R_Bi_G*R_G_W).conjugate();

        geometry_msgs::TransformStamped body;
        body.header.stamp = ros::Time::now();
        body.header.frame_id = "world";
        body.child_frame_id = "gt";
        body.transform.translation.x = body_position(0);
        body.transform.translation.y = body_position(1);
        body.transform.translation.z = body_position(2);
        body.transform.rotation.x = gt_orientation.x();
        body.transform.rotation.y = gt_orientation.y();
        body.transform.rotation.z = gt_orientation.z();
        body.transform.rotation.w = gt_orientation.w();
        broadcaster.sendTransform(body);
    }

    return;

    const CameraPoseBuffer& buffer = tonav_->state().poses();
    if (!buffer.empty()) {
        for (std::size_t i = 0; i < buffer.size(); ++i) {
            const CameraPose& pose = buffer[i];
            
            Quaternion orientation = pose.getCameraOrientationInGlobalFrame(tonav_->filter());
            Eigen::Vector3d position = pose.getCameraPositionInGlobalFrame(tonav_->filter()) / factor_;
            geometry_msgs::TransformStamped body;
            body.header.stamp = ros::Time::now();
            body.header.frame_id = "world";
            body.child_frame_id = "tonavkitti_camera_pose_" + std::to_string(pose.getCameraPoseId());
            body.transform.translation.x = position(0);
            body.transform.translation.y = position(1);
            body.transform.translation.z = position(2);
            body.transform.rotation.x = orientation.x();
            body.transform.rotation.y = orientation.y();
            body.transform.rotation.z = orientation.z();
            body.transform.rotation.w = orientation.w();
            broadcaster.sendTransform(body);
        }
    }
}

void TonavKitti::publishPointCloud(ros::Publisher& publisher) {
    const std::vector<Eigen::Vector3d>& pts = tonav_->featurePointCloud();
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "world";
    cloud.points.resize(pts.size());
    
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(pts.size());
    
    for (std::size_t i = 0; i < pts.size(); ++i) {
        cloud.points[i].x = pts[i](0) / factor_;
        cloud.points[i].y = pts[i](1) / factor_;
        cloud.points[i].z = pts[i](2) / factor_;
        cloud.channels[0].values[i] = 1;
    }
    publisher.publish(cloud);
}

double TonavKitti::deg2rad(double deg) const {
    return (deg * M_PI / 180.0);
}

double TonavKitti::distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) const {
    double lat1r, lon1r, lat2r, lon2r, u, v;
    lat1r = deg2rad(lat1d);
    lon1r = deg2rad(lon1d);
    lat2r = deg2rad(lat2d);
    lon2r = deg2rad(lon2d);
    u = sin((lat2r - lat1r)/2);
    v = sin((lon2r - lon1r)/2);
    return 2.0 * TonavKitti::earthRadiusM * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

Eigen::Matrix3d TonavKitti::rotx(double t) const {
    double c = std::cos(t);
    double s = std::sin(t);
    Eigen::Matrix3d r;
    r << 1, 0, 0, 0, c, -1*s, 0, s, c;
    return r;
}

Eigen::Matrix3d TonavKitti::roty(double t) const {
    double c = std::cos(t);
    double s = std::sin(t);
    Eigen::Matrix3d r;
    r << c, 0, s, 0, 1, 0, -1*s, 0, c;
    return r;
}

Eigen::Matrix3d TonavKitti::rotz(double t) const {
    double c = std::cos(t);
    double s = std::sin(t);
    Eigen::Matrix3d r;
    r << c, -1*s, 0, s, c, 0, 0, 0, 1;
    return r;
}

std::ostream& operator<<(std::ostream& out, const Kitti::OxtsRecord& r) {
    int w = 14;
    return out << "KittiOxtsRecord" << std::endl
    << std::setfill(' ') << std::setw(w) << "lat: " << r.lat << std::endl
    << std::setfill(' ') << std::setw(w) << "lon: " << r.lon << std::endl
    << std::setfill(' ') << std::setw(w) << "alt: " << r.alt << std::endl
    << std::setfill(' ') << std::setw(w) << "roll: " << r.roll << std::endl
    << std::setfill(' ') << std::setw(w) << "pitch: " << r.pitch << std::endl
    << std::setfill(' ') << std::setw(w) << "yaw: " << r.yaw << std::endl
    << std::setfill(' ') << std::setw(w) << "vn: " << r.vn << std::endl
    << std::setfill(' ') << std::setw(w) << "ve: " << r.ve << std::endl
    << std::setfill(' ') << std::setw(w) << "vf: " << r.vf << std::endl
    << std::setfill(' ') << std::setw(w) << "vl: " << r.vl << std::endl
    << std::setfill(' ') << std::setw(w) << "vu: " << r.vu << std::endl
    << std::setfill(' ') << std::setw(w) << "ax: " << r.ax << std::endl
    << std::setfill(' ') << std::setw(w) << "ay: " << r.ay << std::endl
    << std::setfill(' ') << std::setw(w) << "az: " << r.az << std::endl
    << std::setfill(' ') << std::setw(w) << "af: " << r.af << std::endl
    << std::setfill(' ') << std::setw(w) << "al: " << r.al << std::endl
    << std::setfill(' ') << std::setw(w) << "au: " << r.au << std::endl
    << std::setfill(' ') << std::setw(w) << "wx: " << r.wx << std::endl
    << std::setfill(' ') << std::setw(w) << "wy: " << r.wy << std::endl
    << std::setfill(' ') << std::setw(w) << "wz: " << r.wz << std::endl
    << std::setfill(' ') << std::setw(w) << "wf: " << r.wf << std::endl
    << std::setfill(' ') << std::setw(w) << "wl: " << r.wl << std::endl
    << std::setfill(' ') << std::setw(w) << "wu: " << r.wu << std::endl
    << std::setfill(' ') << std::setw(w) << "pos_accuracy: " << r.pos_accuracy << std::endl
    << std::setfill(' ') << std::setw(w) << "vel_accuracy: " << r.vel_accuracy << std::endl
    << std::setfill(' ') << std::setw(w) << "navstat: " << r.navstat << std::endl
    << std::setfill(' ') << std::setw(w) << "numsats: " << r.numsats << std::endl
    << std::setfill(' ') << std::setw(w) << "posmode: " << r.posmode << std::endl
    << std::setfill(' ') << std::setw(w) << "velmode: " << r.velmode << std::endl
    << std::setfill(' ') << std::setw(w) << "orimode: " << r.orimode << std::endl;
}
