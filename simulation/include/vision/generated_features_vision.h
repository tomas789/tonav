//
// Created by Tomas Krejci on 10/14/17.
//

#ifndef TONAV_GENERATED_FEATURES_VISION_H
#define TONAV_GENERATED_FEATURES_VISION_H

#include <opencv2/features2d.hpp>
#include <random>

#include "../vision.h"
#include "../run_loop_callback.h"

class VioSimulation;

class GeneratedFeaturesVision: public Vision, public RunLoopCallback {
public:
    static std::unique_ptr<Vision> load(SimSetup* sim_setup, const json& j);
    
    void initialize(VioSimulation *simulation);
    
    void runLoopCallback(double time);
    
    cv::Matx33d getCameraCalibrationMatrix() const;
    
    std::vector<Eigen::Vector3d> getFeaturesInView() const;
    
    cv::Ptr<cv::Feature2D> getFeature2D();
    
    Eigen::Vector2d getFocalLength() const;
    Eigen::Vector2d getOpticalCenter() const;
    Eigen::Vector3d getRadialDistortion() const;
    Eigen::Vector2d getTangentialDistortion() const;
    
    virtual ~GeneratedFeaturesVision();

protected:
    GeneratedFeaturesVision(SimSetup *sim_setup);
    
    class VirtualFeature2D: public cv::Feature2D {
    public:
        VirtualFeature2D(GeneratedFeaturesVision& vision);
    
        void detectAndCompute(
            cv::InputArray image,
            cv::InputArray mask,
            std::vector<cv::KeyPoint>& keypoints,
            cv::OutputArray descriptors,
            bool useProvidedKeypoints=false
        );
        
        virtual ~VirtualFeature2D();
    
    private:
        GeneratedFeaturesVision &vision_;
    };
    
    struct VirtualKeyPoint {
        std::size_t feature_id;
        Eigen::Vector2d pt_last;
        Eigen::Vector3d p_f_G;
        cv::Mat descriptor;
    };
    
    VirtualKeyPoint generateVisibleFeature(Eigen::Vector3d p_C_G, tonav::Quaternion q_C_G);
    Eigen::Vector2d projectFeature(Eigen::Vector3d p_f_G, Eigen::Vector3d p_C_G, tonav::Quaternion q_C_G) const;
    bool isFeatureInView(Eigen::Vector3d p_f_G, Eigen::Vector3d p_C_G, tonav::Quaternion q_C_G) const;
    
    VioSimulation *simulation_;
    
    Eigen::Vector2d focal_length_;
    Eigen::Vector2d optical_center_;
    Eigen::Vector2d image_dimensions_;
    int minimum_number_of_features_;
    int maximum_number_of_features_;
    
    cv::Ptr<VirtualFeature2D> feature2d_;
    
    std::default_random_engine random_generator_;
    
    std::size_t feature_counter_;
    
    std::vector<VirtualKeyPoint> visible_features_;
};

#endif //TONAV_GENERATED_FEATURES_VISION_H
