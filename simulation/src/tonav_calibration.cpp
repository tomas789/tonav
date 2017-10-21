//
// Created by Tomas Krejci on 10/21/17.
//

#include "tonav_calibration.h"

std::shared_ptr<TonavCalibration> TonavCalibration::prepare(SimSetup *sim_setup) {
    std::shared_ptr<TonavCalibration> calibration(new TonavCalibration);
    
    return calibration;
}

TonavCalibration::TonavCalibration() = default;

