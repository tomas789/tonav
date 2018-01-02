//
// Created by Tomas Krejci on 10/7/17.
//

#ifndef TONAV_SIM_SETUP_COMPONENT_H
#define TONAV_SIM_SETUP_COMPONENT_H

#include <string>

class RunLoop;
class SimSetup;
class VioSimulation;

class SimSetupComponent {
protected:
    SimSetupComponent(SimSetup* sim_setup);
    
public:
    virtual void initialize(VioSimulation *simulation) = 0;
        
    virtual ~SimSetupComponent();
    
protected:
    SimSetup *sim_setup_;
    std::string component_name_;
    
};

#endif //TONAV_SIM_SETUP_COMPONENT_H
