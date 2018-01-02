//
// Created by Tomas Krejci on 26/12/17.
//

#ifndef TONAV_HELPER_H
#define TONAV_HELPER_H

#include <json.hpp>
#include <Eigen/Core>
#include <memory>

#include "sim_setup_component.h"
#include "trajectory.h"

using json = nlohmann::json;

class Helper: protected SimSetupComponent {
public:
    static std::unique_ptr<Helper> load(SimSetup *sim_setup, const json& j);
    
    virtual void initialize(VioSimulation *simulation) = 0;
    
    virtual ~Helper();
    
protected:
    Helper(SimSetup *sim_setup);
    
};

#endif //TONAV_HELPER_H

