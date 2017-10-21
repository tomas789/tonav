//
// Created by Tomas Krejci on 10/14/17.
//

#include "vision/generated_features_vision.h"
#include "vision.h"
#include "vio_simulation.h"
#include "run_loop.h"

std::unique_ptr<Vision> GeneratedFeaturesVision::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<GeneratedFeaturesVision> vision(new GeneratedFeaturesVision(sim_setup));
    
    vision->minimum_number_of_features_ = j.at("minimum_number_of_features");
    vision->maximum_number_of_features_ = j.at("maximum_number_of_features");
    
    json camera_model = j.at("camera_model");
    
    vision->focal_length_ << camera_model.at("fx"), camera_model.at("fy");
    vision->optical_center_ << camera_model.at("cx"), camera_model.at("cy");
    vision->image_dimensions_ << camera_model.at("width"), camera_model.at("height");
    
    return std::move(vision);
}

void GeneratedFeaturesVision::initialize(VioSimulation *simulation) {
    simulation_ = simulation;
    simulation_->getRunLoop().registerCallback(0, this);
}

void GeneratedFeaturesVision::runLoopCallback(double time) {
    
    std::cout << "Vision: Time: " << time << ", Next Time: " << (time+1.0/update_frequency_) << std::endl;
    simulation_->getRunLoop().registerCallback(time+1.0/update_frequency_, this);
}

cv::Matx33d GeneratedFeaturesVision::getCameraCalibrationMatrix() const {
    return cv::Matx33d(
        focal_length_(0), 0, optical_center_(0),
        0, focal_length_(1), optical_center_(1),
        0, 0, 1
    );
}

GeneratedFeaturesVision::~GeneratedFeaturesVision() = default;
GeneratedFeaturesVision::GeneratedFeaturesVision(SimSetup *sim_setup) : Vision(sim_setup) { }
