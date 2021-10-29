#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atank/Command.h"

#include <thread>
#include <signal.h>

void mySignalHandler(int sig) {
    ROS_INFO("Signal handler is called. Server will be terminated.");
    ros::shutdown();
    exit(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_recoder");
    ros::NodeHandle n;

    ROS_INFO("[CameraRecoder] initialization is successful.");

    signal(SIGINT, mySignalHandler);

    // "command": topic name
    ros::ServiceClient client = n.serviceClient<atank::Command>("command");

    atank::Command tankCmd;
    tankCmd.request.cmd = std::string("version");

    if (client.call(tankCmd)) {
        if (tankCmd.response.ack == true) {
            ROS_INFO("[CameraRecoder] connection is successful.");
            ROS_INFO("[CameraRecoder] ack: %s", tankCmd.response.log.c_str());
        }
        else {
            ROS_INFO("[CameraRecoder] connection is failed.");
        }
    }
    else {
        ROS_INFO("[CameraRecoder] failed to call service.");
    }

    return 0;
}
