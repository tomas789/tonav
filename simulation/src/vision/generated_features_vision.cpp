//
// Created by Tomas Krejci on 10/14/17.
//

#include "vision/generated_features_vision.h"
#include "vision.h"
#include "vio_simulation.h"
#include "run_loop.h"

std::unique_ptr<Vision> GeneratedFeaturesVision::load(SimSetup *sim_setup, const json &j) {
    std::unique_ptr<GeneratedFeaturesVision> vision(new GeneratedFeaturesVision(sim_setup));
    
    json minimum_number_of_features_json = j.at("minimum_number_of_features");
    if (!minimum_number_of_features_json.is_number())
        throw "'minimum_number_of_features' has to be a number.";
    vision->minimum_number_of_features_ = minimum_number_of_features_json;
    
    json maximum_number_of_features_json = j.at("maximum_number_of_features");
    if (!maximum_number_of_features_json.is_number())
        throw "'maximum_number_of_features' has to be a number.";
    vision->maximum_number_of_features_ = maximum_number_of_features_json;
    
    json update_frequency_json = j.at("update_frequency");
    if (!update_frequency_json.is_number())
        throw "'update_frequency' has to be a number.";
    vision->update_frequency_ = update_frequency_json;
    if (vision->update_frequency_ <= 0.0)
        throw "'update_frequency' has to be positive number.";
    
    json camera_model = j.at("camera_model");
    
    json fx_json = camera_model.at("fx");
    if (!fx_json.is_number())
        throw "'camera_model.fx' has to be a number.";
    double fx = fx_json;
    
    json fy_json = camera_model.at("fy");
    if (!fy_json.is_number())
        throw "'camera_model.fy' has to be a number.";
    double fy = fy_json;
    
    vision->focal_length_ << fx, fy;
    
    json cx_json = camera_model.at("cx");
    if (!cx_json.is_number())
        throw "'camera_model.cx' has to be a number.";
    double cx = cx_json;
    
    json cy_json = camera_model.at("cy");
    if (!cy_json.is_number())
        throw "'camera_model.cy' has to be a number.";
    double cy = cy_json;
    
    vision->optical_center_ << cx, cy;
    
    json width_json = camera_model.at("width");
    if (!width_json.is_number())
        throw "'camera_model.width' has to be a number.";
    double width = width_json;
    
    json height_json = camera_model.at("height");
    if (!height_json.is_number())
        throw "'camera_model.height' has to be a number.";
    double height = height_json;
    
    vision->image_dimensions_ << width, height;
    
    return std::move(vision);
}

void GeneratedFeaturesVision::initialize(VioSimulation *simulation) {
    simulation_ = simulation;
    simulation_->getRunLoop().registerCallback(0, this);
}

void GeneratedFeaturesVision::runLoopCallback(float time) {
    
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

float GeneratedFeaturesVision::getUpdateFrequency() const {
    return update_frequency_;
}

GeneratedFeaturesVision::~GeneratedFeaturesVision() = default;
GeneratedFeaturesVision::GeneratedFeaturesVision(SimSetup *sim_setup) : Vision(sim_setup) { }
