#include <iostream>
#include <tonav.h>

#include "sim_setup.h"
#include "tonav_calibration.h"
#include "vio_simulation.h"

int main(int argc, char* argv[]) {
    std::vector<std::string> args(argv, argv+argc);
    if (args.size() != 2) {
        std::cerr << "Usage: tonav-simulation <sim_setup.json>" << std::endl;
        return EXIT_FAILURE;
    }

    std::shared_ptr<SimSetup> sim_setup = SimSetup::load(args[1]);
    std::shared_ptr<TonavCalibration> calibration = TonavCalibration::prepare(sim_setup.get());
    tonav::Tonav tonav(calibration, Eigen::Vector3d::Zero());
    VioSimulation sim;
    sim.run(sim_setup);

    return EXIT_SUCCESS;
}