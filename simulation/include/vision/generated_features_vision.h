//
// Created by Tomas Krejci on 10/14/17.
//

#ifndef TONAV_GENERATED_FEATURES_VISION_H
#define TONAV_GENERATED_FEATURES_VISION_H

#include <opencv2/features2d.hpp>

#include "../vision.h"
#include "../run_loop_callback.h"

class VioSimulation;

class GeneratedFeaturesVision: public Vision, public RunLoopCallback {
public:
    static std::unique_ptr<Vision> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    void runLoopCallback(double time);
    
    cv::Matx33d getCameraCalibrationMatrix() const;
    
    cv::Feature2D& getFeature2D();
    
    virtual ~GeneratedFeaturesVision();

protected:
    GeneratedFeaturesVision(SimSetup *sim_setup);
    
    class VirtualFeatures: public cv::Feature2D {
    public:
        VirtualFeatures(GeneratedFeaturesVision& vision);
        
        virtual ~VirtualFeatures();
    
    private:
        GeneratedFeaturesVision &vision_;
    };
    
    VioSimulation *simulation_;
    
    Eigen::Vector2d focal_length_;
    Eigen::Vector2d optical_center_;
    Eigen::Vector2d image_dimensions_;
    int minimum_number_of_features_;
    int maximum_number_of_features_;
    
    VirtualFeatures feature2d_;
    
    std::vector<Eigen::Vector3d> visible_features_;
};

#endif //TONAV_GENERATED_FEATURES_VISION_H
