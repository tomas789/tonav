#include "vision/kitti_vision.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "vio_simulation.h"

std::unique_ptr<Vision> KittiVision::load(SimSetup *sim_setup, const KittiLoaderHelper &j) {
    std::unique_ptr<KittiVision> vision(new KittiVision(sim_setup));
    
    vision->true_features_ = cv::xfeatures2d::SIFT::create(200);
    
    return vision;
}

void KittiVision::initialize(VioSimulation *simulation) {
    simulation_ = simulation;
    if (!timestamps_.empty()) {
        simulation_->getRunLoop().registerCallback(timestamps_.front(), this);
        timestamps_.pop_front();
    }
}

void KittiVision::runLoopCallback(double time) {
    std::string file_name = camera_images_[time];
    cv::Mat frame = cv::imread(file_name);
    
    cv::FileStorage fs(file_name + ".opencv", cv::FileStorage::READ);
    if (fs.isOpened()) {
        cv::FileNode kpts_file_node = fs["keypoints"];
        cv::FileNode des_file_node = fs["descriptors"];
        cv::read(kpts_file_node, visible_features_);
        cv::read(des_file_node, visible_features_descriptors_);
        fs.release();
    } else {
        true_features_->detectAndCompute(frame, cv::Mat(), visible_features_, visible_features_descriptors_);
        cv::FileStorage fs_write(file_name + ".opencv", cv::FileStorage::WRITE);
        cv::write(fs_write, "keypoints", visible_features_);
        cv::write(fs_write, "descriptors", visible_features_descriptors_);
        fs_write.release();
    }
    
    simulation_->cameraCallback(time, frame);
    
    if (!timestamps_.empty()) {
        simulation_->getRunLoop().registerCallback(timestamps_.front(), this);
        timestamps_.pop_front();
    }
}

cv::Matx33d KittiVision::getCameraCalibrationMatrix() const {
    return cv::Matx33d(
        focal_length_(0), 0, optical_center_(0),
        0, focal_length_(1), optical_center_(1),
        0, 0, 1
    );
}

std::vector<Eigen::Vector3d> KittiVision::getFeaturesInView() const {
    return std::vector<Eigen::Vector3d>();
}

cv::Ptr<cv::Feature2D> KittiVision::getFeature2D() {
    return feature2d_;
}

Eigen::Vector2d KittiVision::getFocalLength() const {
    return focal_length_;
}

Eigen::Vector2d KittiVision::getOpticalCenter() const {
    return optical_center_;
}

Eigen::Vector3d KittiVision::getRadialDistortion() const {
    return Eigen::Vector3d::Zero();
}

Eigen::Vector2d KittiVision::getTangentialDistortion() const {
    return Eigen::Vector2d::Zero();
}

KittiVision::KittiVision(SimSetup* sim_setup)
: Vision(sim_setup), feature2d_(new ProxyFeature2D(*this)) { }
KittiVision::~KittiVision() = default;

KittiVision::ProxyFeature2D::ProxyFeature2D(KittiVision &vision)
: vision_(vision) { }

void KittiVision::ProxyFeature2D::detectAndCompute(
    cv::InputArray image,
    cv::InputArray mask,
    std::vector<cv::KeyPoint>& keypoints,
    cv::OutputArray descriptors,
    bool useProvidedKeypoints
) {
    if (useProvidedKeypoints) {
        throw std::runtime_error(
            "KittiVision::ProxyFeature2D does not support calling detectAndCompute "
            "with useProvidedKeypoints set to true."
        );
    }
    
    if (!keypoints.empty()) {
        throw std::runtime_error("Cannot call detectAndCompute with non-empty keypoints vector.");
    }
    
    keypoints = std::vector<cv::KeyPoint>(
        std::begin(vision_.visible_features_),
        std::end(vision_.visible_features_)
    );
    vision_.visible_features_descriptors_.copyTo(descriptors);
}

KittiVision::ProxyFeature2D::~ProxyFeature2D() = default;
