#include "helper.h"
#include "helpers/kitti_loader_helper.h"

std::unique_ptr<Helper> Helper::load(SimSetup *sim_setup, const json &j) {
    std::string type = j.at("type").get<std::string>();
    json params = j.at("params");
    
    std::unique_ptr<Helper> helper;
    
    if (type == "kitti_loader") {
        helper = KittiLoaderHelper::load(sim_setup, params);
    } else {
        throw std::runtime_error("Unknown imu type.");
    }
    
    return helper;
}

Helper::Helper(SimSetup *sim_setup) : SimSetupComponent(sim_setup) { }
Helper::~Helper() = default;
