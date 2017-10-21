//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_VISION_H
#define TONAV_VISION_H

#include <Eigen/Core>
#include <tonav.h>
#include <json.hpp>

#include "sim_setup_component.h"

class SimSetup;

using json = nlohmann::json;

class Vision: protected SimSetupComponent {
public:
    static std::unique_ptr<Vision> load(SimSetup *sim_data, const json& j);
    
    virtual void initialize(VioSimulation *simulation) = 0;
    
    virtual cv::Matx33d getCameraCalibrationMatrix() const = 0;
    double getUpdateFrequency() const;
    
    virtual ~Vision();

protected:
    Vision(SimSetup *sim_setup);
    
    double update_frequency_;
};

#endif //TONAV_TRAJECTORY_H
