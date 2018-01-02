//
// Created by Tomas Krejci on 26/12/17.
//

#ifndef TONAV_KITTI_VISION_H
#define TONAV_KITTI_VISION_H

#include <opencv2/features2d.hpp>
#include <random>
#include <forward_list>

#include "../vision.h"
#include "../run_loop_callback.h"

class VioSimulation;
class KittiLoaderHelper;

class KittiVision: public Vision, public RunLoopCallback {
public:
    friend class KittiLoaderHelper;
    
    static std::unique_ptr<Vision> load(SimSetup* sim_setup, const KittiLoaderHelper& j);
    
    void initialize(VioSimulation *simulation);
    
    void runLoopCallback(double time);
    
    cv::Matx33d getCameraCalibrationMatrix() const;
    
    std::vector<Eigen::Vector3d> getFeaturesInView() const;
    
    cv::Ptr<cv::Feature2D> getFeature2D();
    
    Eigen::Vector2d getFocalLength() const;
    Eigen::Vector2d getOpticalCenter() const;
    Eigen::Vector3d getRadialDistortion() const;
    Eigen::Vector2d getTangentialDistortion() const;
    
    virtual ~KittiVision();
    
protected:
    KittiVision(SimSetup *sim_setup);
    
    class ProxyFeature2D: public cv::Feature2D {
    public:
        ProxyFeature2D(KittiVision& vision);
        
        void detectAndCompute(
            cv::InputArray image,
            cv::InputArray mask,
            std::vector<cv::KeyPoint>& keypoints,
            cv::OutputArray descriptors,
            bool useProvidedKeypoints=false
        );
        
        virtual ~ProxyFeature2D();
        
    private:
        KittiVision &vision_;
    };
    
    VioSimulation* simulation_;
    
    std::forward_list<double> timestamps_;
    Eigen::Vector2d focal_length_;
    Eigen::Vector2d optical_center_;
    std::map<double, std::string> camera_images_;
    
    cv::Ptr<cv::Feature2D> true_features_;
    cv::Ptr<ProxyFeature2D> feature2d_;
    std::vector<cv::KeyPoint> visible_features_;
    cv::Mat visible_features_descriptors_;
};

#endif //TONAV_KITTI_VISION_H

