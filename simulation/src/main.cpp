#include <iostream>
#include <tonav.h>

#include "sim_setup.h"
#include "vio_simulation.h"

int main(int argc, char* argv[]) {
    std::vector<std::string> args(argv, argv+argc);
    if (args.size() != 2) {
        std::cerr << "Usage: tonav-simulation <sim_setup.json>" << std::endl;
        return EXIT_FAILURE;
    }

    std::shared_ptr<SimSetup> sim_setup = SimSetup::load(args[1]);
    
    VioSimulation sim;
    // sim.setHeadless();
    sim.setSimulationLength(80);
    sim.run(sim_setup);

    return EXIT_SUCCESS;
}
