//
// Created by Tomas Krejci on 10/21/17.
//

#ifndef TONAV_TONAV_CALIBRATION_H
#define TONAV_TONAV_CALIBRATION_H

#include <tonav.h>
#include <memory>

#include "sim_setup.h"

class TonavCalibration: public tonav::Calibration {
public:
    static std::shared_ptr<TonavCalibration> prepare(SimSetup* sim_setup);

private:
    TonavCalibration();
};

#endif //TONAV_TONAV_CALIBRATION_H
