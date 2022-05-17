#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atank/LidarFrame.h"
#include "lidar_mapper.h"

#include <thread>
#include <signal.h>

void mySignalHandler(int sig) {
    ROS_INFO("Signal handler is called. Server will be terminated.");
    ros::shutdown();
    exit(1);
}

void lidar_frame_callback(const atank::LidarFrame &msg) {
    ROS_INFO("LidarFrame: frame length[%3d]", msg.size);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_mapper");
    ros::NodeHandle n;

    signal(SIGINT, mySignalHandler);

    // "command": topic name
    ros::Subscriber sub = n.subscribe("lidar_frame", 1000, lidar_frame_callback);
    ROS_INFO("[CMD SERVER] 'subscriber' is ready to process.");

    ros::spin();

    return 0;
}
