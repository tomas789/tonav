#include "tonav_ros.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "tonav_ros");    
	TonavRos tonav_ros;
    return tonav_ros.run(argc, argv);
}
