//
// Created by Tomas Krejci on 10/14/17.
//

#include "vision/generated_features_vision.h"

#include <algorithm>
#include <vector>

#include "vision.h"
#include "vio_simulation.h"
#include "run_loop.h"

std::unique_ptr<Vision> GeneratedFeaturesVision::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<GeneratedFeaturesVision> vision(new GeneratedFeaturesVision(sim_setup));
    
    vision->minimum_number_of_features_ = j.at("minimum_number_of_features");
    vision->maximum_number_of_features_ = j.at("maximum_number_of_features");
    if (vision->maximum_number_of_features_ <= vision->minimum_number_of_features_) {
        throw std::runtime_error("'maximum_number_of_features' has to be greater then 'minimum_number_of_features'");
    }
    
    json camera_model = j.at("camera_model");
    
    vision->focal_length_ << camera_model.at("fx"), camera_model.at("fy");
    vision->optical_center_ << camera_model.at("cx"), camera_model.at("cy");
    vision->image_dimensions_ << camera_model.at("width"), camera_model.at("height");
    
    return std::move(vision);
}

void GeneratedFeaturesVision::initialize(VioSimulation *simulation) {
    simulation_ = simulation;
    feature_counter_ = 0;
    visible_features_.clear();
    random_generator_.seed(42);
    
    simulation_->getRunLoop().registerCallback(0, this);
}

void GeneratedFeaturesVision::runLoopCallback(double time) {
    const Trajectory& trajectory = sim_setup_->getTrajectory();
    Eigen::Vector3d p_C_G = trajectory.getBodyPositionInGlobalFrame(time);
    tonav::Quaternion q_C_G = trajectory.getGlobalToCameraFrameRotation(time);
    
    std::vector<VirtualKeyPoint> current_visible_features;
    for (int i = 0; i < visible_features_.size(); ++i) {
        const VirtualKeyPoint& feature = visible_features_[i];
        Eigen::Vector3d p_f_G = feature.p_f_G;
        bool is_in_view = isFeatureInView(p_f_G, p_C_G, q_C_G);
        
        if (is_in_view) {
            current_visible_features.push_back(feature);
            current_visible_features.back().pt_last = projectFeature(feature.p_f_G, p_C_G, q_C_G);
        }
    }
    
    if (current_visible_features.size() < minimum_number_of_features_) {
        std::size_t features_to_generate = maximum_number_of_features_ - visible_features_.size();
        
        for (int i = 0; i < features_to_generate; ++i) {
            VirtualKeyPoint feature = generateVisibleFeature(p_C_G, q_C_G);
            current_visible_features.push_back(feature);
        }
    }
    
    visible_features_ = current_visible_features;
    
    cv::Mat frame(cv::Size(image_dimensions_(0), image_dimensions_(1)), CV_8UC3, cv::Scalar(0));
    simulation_->cameraCallback(time, frame);
    
    simulation_->getRunLoop().registerCallback(time+1.0/update_frequency_, this);
}

cv::Matx33d GeneratedFeaturesVision::getCameraCalibrationMatrix() const {
    return cv::Matx33d(
        focal_length_(0), 0, optical_center_(0),
        0, focal_length_(1), optical_center_(1),
        0, 0, 1
    );
}

std::vector<Eigen::Vector3d> GeneratedFeaturesVision::getFeaturesInView() const {
    std::vector<Eigen::Vector3d> features_in_view;
    for (const auto& item : visible_features_) {
        features_in_view.push_back(item.p_f_G);
    }
    return features_in_view;
}

cv::Ptr<cv::Feature2D> GeneratedFeaturesVision::getFeature2D() {
    return feature2d_;
}

Eigen::Vector2d GeneratedFeaturesVision::getFocalLength() const {
    return focal_length_;
}

Eigen::Vector2d GeneratedFeaturesVision::getOpticalCenter() const {
    return optical_center_;
}

Eigen::Vector3d GeneratedFeaturesVision::getRadialDistortion() const {
    return Eigen::Vector3d::Zero();
}

Eigen::Vector2d GeneratedFeaturesVision::getTangentialDistortion() const {
    return Eigen::Vector2d::Zero();
}

GeneratedFeaturesVision::~GeneratedFeaturesVision() = default;

GeneratedFeaturesVision::GeneratedFeaturesVision(SimSetup *sim_setup)
    : Vision(sim_setup), feature2d_(new VirtualFeature2D(*this)) {
    
}

GeneratedFeaturesVision::VirtualFeature2D::VirtualFeature2D(GeneratedFeaturesVision &vision)
    : vision_(vision) {
    
}

void GeneratedFeaturesVision::VirtualFeature2D::detectAndCompute(
    cv::InputArray image,
    cv::InputArray mask,
    std::vector<cv::KeyPoint>& keypoints,
    cv::OutputArray descriptors,
    bool useProvidedKeypoints
) {
    if (useProvidedKeypoints) {
        throw std::runtime_error(
            "GeneratedFeaturesVision::VirtualFeatures does not support calling detectAndCompute "
            "with useProvidedKeypoints set to true."
        );
    }
    
    if (!keypoints.empty()) {
        throw std::runtime_error("Cannot call detectAndCompute with non-empty keypoints vector.");
    }
    
    descriptors.create((int)vision_.visible_features_.size(), 128, CV_32F);
    cv::Mat descriptors_mat = descriptors.getMat();
    for (int i = 0; i < vision_.visible_features_.size(); ++i) {
        const VirtualKeyPoint& kpt = vision_.visible_features_[i];
        Eigen::Vector2d pt = kpt.pt_last;
        keypoints.emplace_back(cv::Point2f(pt(0), pt(1)), 1);
        kpt.descriptor.copyTo(descriptors_mat.row(i));
    }
}

GeneratedFeaturesVision::VirtualFeature2D::~VirtualFeature2D() = default;

GeneratedFeaturesVision::VirtualKeyPoint GeneratedFeaturesVision::generateVisibleFeature(Eigen::Vector3d p_C_G, tonav::Quaternion q_C_G) {
    std::uniform_real_distribution<> x_dist(0.2*image_dimensions_(0), 0.8*image_dimensions_(0));
    std::uniform_real_distribution<> y_dist(0.2*image_dimensions_(1), 0.8*image_dimensions_(1));
    std::gamma_distribution<> depth_dist(9.0, 0.5);
    double x = x_dist(random_generator_);
    double y = y_dist(random_generator_);
    double depth = depth_dist(random_generator_);
    
    Eigen::Vector3d p_f_C;
    p_f_C <<
        depth*(x - optical_center_(0))/focal_length_(0),
        depth*(y - optical_center_(1))/focal_length_(1),
        depth;
    
    Eigen::Matrix3d R_G_C = q_C_G.conjugate().toRotationMatrix();
    Eigen::Vector3d p_f_G = R_G_C*p_f_C + p_C_G;
    
    VirtualKeyPoint feature;
    feature.feature_id = feature_counter_;
    feature.pt_last << x, y;
    feature.p_f_G = p_f_G;
    feature.descriptor.create(1, 128, CV_32F);
    
    std::uniform_real_distribution<float> desc_dist(0, 1);
    for (int i = 0; i < 128; ++i) {
        feature.descriptor.at<float>(0, i) = desc_dist(random_generator_);
    }
    
    feature_counter_ += 1;
    
    return feature;
}

Eigen::Vector2d GeneratedFeaturesVision::projectFeature(Eigen::Vector3d p_f_G, Eigen::Vector3d p_C_G, tonav::Quaternion q_C_G) const {
    Eigen::Matrix3d R_C_G = q_C_G.toRotationMatrix();
    Eigen::Vector3d p_f_C = R_C_G*(p_f_G - p_C_G);
    double x = focal_length_(0)*p_f_C(0)/p_f_C(2) + optical_center_(0);
    double y = focal_length_(1)*p_f_C(1)/p_f_C(2) + optical_center_(1);
    Eigen::Vector2d pt;
    pt << x, y;
    return pt;
}

bool GeneratedFeaturesVision::isFeatureInView(Eigen::Vector3d p_f_G, Eigen::Vector3d p_C_G, tonav::Quaternion q_C_G) const {
    Eigen::Vector2d pt = projectFeature(p_f_G, p_C_G, q_C_G);
    double x = pt(0);
    double y = pt(1);
    bool x_in_view = x >= 0 && x <= image_dimensions_(0);
    bool y_in_view = y >= 0 && y <= image_dimensions_(1);
    return x_in_view && y_in_view;
}
