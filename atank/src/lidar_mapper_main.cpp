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
    LidarMapper *lmapper = LidarMapper::GetInstance();
    ROS_INFO("LidarFrame: frame [%3d, %d]", msg.size, (int)msg.data.size());
    
    // dump lidar raw byte date into a file.
    lmapper->dumpRawByte(msg.data);
    // processing lidar raw frame(byte stream)
    lmapper->procRawLidarFrame(msg.data);
}

void ros_spin(void) {
    ros::spin();
    return;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_mapper_main");
    ros::NodeHandle n;

    signal(SIGINT, mySignalHandler);

    // "command": topic name
    ros::Subscriber sub = n.subscribe("lidar_frame", 1000, lidar_frame_callback);
    ROS_INFO("[CMD SERVER] 'subscriber' is ready to process.");

    std::thread _spin(ros_spin);
    std::thread _window(initOpenGL, argc, argv);

    _spin.join();
    _window.join();

    // TEST CODE
    //lmapper.TEST_procRawLidarFrame();

    //ros::spin();

    return 0;
}
