#include "cms/PingResult.h"
#include <ros/ros.h>
#include <unordered_map>
#include <topic_tools/shape_shifter.h>


// The topics that we want to keep an eye on.
// This node will subscribe to these topics, count the messages and send the statistics to /cms/statistics
std::vector<std::pair<std::string, std::string>> devices = {
    std::make_pair("192.168.1.11", "Camera Quad 1"),
    std::make_pair("192.168.1.12", "Camera Quad 2"),
    std::make_pair("192.168.1.13", "Camera Quad 3"),
    std::make_pair("192.168.1.14", "Camera Quad 4"),
    std::make_pair("192.168.1.19", "Camera VIO"),
    std::make_pair("192.168.1.201", "Lidar"),
    std::make_pair("192.168.1.6", "Groundstation router")
};

ros::Publisher pings_publisher;

void timerCallback(const ros::TimerEvent&) {
    for (auto const& device : devices) {
        if(ros::isShuttingDown()) {
            return;
        }
        auto msg = cms::PingResult();
        msg.success = system(("ping -c 1 -w 1 -W 1 " + device.first + " > /dev/null 2>&1").c_str()) == 0;
        msg.ip = device.first;
        msg.friendlyname = device.second;
        pings_publisher.publish(msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cms_pings");
    ros::NodeHandle nh;

    pings_publisher = nh.advertise<cms::PingResult>("/cms/pings", 10);
    ros::Timer timer = nh.createTimer(ros::Duration(3), timerCallback);

    ros::spin();
    return 0;
}