#include "cms/Statistic.h"
#include "cms/Statistics.h"
#include <ros/ros.h>
#include <unordered_map>
#include <topic_tools/shape_shifter.h>

using topic_tools::ShapeShifter;

// The topics that we want to keep an eye on.
// This node will subscribe to these topics, count the messages and send the statistics to /cms/statistics
std::vector<std::string> topics = {
    "/vehicle_interface/to/TrajectorySetpoints",
    "/camera/quad1/image_raw",
    "/camera/quad2/image_raw",
    "/camera/quad3/image_raw",
    "/camera/quad4/image_raw",
    "/velodyne_packets",
    "/velodyne_points",
    "/camera/image_raw",
    "/mavros/imu/data_raw",
    "/mavros/local_position/pose",
};

// Amount of messages on every topic since last sending it
std::unordered_map<std::string, int> topicCounter;

ros::Publisher statistics_pub;

void topicCallback(const ShapeShifter::ConstPtr& msg, const std::string &topic_name) {
    topicCounter[topic_name]++;
}

void timerCallback(const ros::TimerEvent&) {
    std::vector<cms::Statistic> statisticsMessages = {};
    for (auto const& topic_name : topics) {
        auto msg = cms::Statistic();
        msg.topic_name = topic_name;
        msg.traffic = topicCounter[topic_name];
        statisticsMessages.push_back(msg);
        topicCounter[topic_name] = 0;
    }
    auto msg = cms::Statistics();
    msg.statistics = statisticsMessages;
    statistics_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cms_statistics");
    ros::NodeHandle nh;

    statistics_pub = nh.advertise<cms::Statistics>("/cms/statistics", 10);

    std::vector<ros::Subscriber> subscribers = {};

    for (auto const& topic_name : topics) {
        topicCounter[topic_name] = 0;

        //ros docs: Who is afraid of lambdas and boost::functions?
        //sijmen: ✋
        boost::function<void(const ShapeShifter::ConstPtr&)> callback;
        callback = [topic_name](const ShapeShifter::ConstPtr& msg) {
            topicCallback(msg, topic_name);
        };

        subscribers.push_back(nh.subscribe(topic_name, 10, callback));
    }

    ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);

    ros::spin();
    return 0;
}