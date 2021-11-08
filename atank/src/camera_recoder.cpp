#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atank/Command.h"

#include <chrono>
#include <thread>
#include <queue>
#include <signal.h>
#include <opencv2/opencv.hpp>

void mySignalHandler(int sig) {
    ROS_INFO("Signal handler is called. Server will be terminated.");
    ros::shutdown();
    exit(1);
}

#define TIME_TO_SLEEP   1000

class CommandProcessor {
public:
    CommandProcessor(ros::NodeHandle &node) {
        node_ = node;
        client_ = node_.serviceClient<atank::Command>("command");
    }
    ~CommandProcessor(void) {
    }
    int size(void) {
        return cmd_queue_.size();
    }
    void pushCommand(const char *command) {
        cmd_queue_.push(std::string(command));
    }
    void executeCommand(void) {
        std::string cmd = cmd_queue_.front();
        cmd_queue_.pop();

        if (cmd.compare("wait_10ms") == 0) {
            std::cout << "[DEBUG] wait 10 msec" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(TIME_TO_SLEEP));
        }
        else if (cmd.compare("left_turn") == 0) {
            requestRosService("motor.left.fw");
        }
        else if (cmd.compare("right_turn") == 0) {
            requestRosService("motor.right.fw");
        }
        else if (cmd.compare("stop") == 0) {
            requestRosService("motor.both.stop");
        }
        else if (cmd.compare("version") == 0) {
            requestRosService("version");
        }
        else {
            std::cerr << "[ERROR] unknown instruction!" << std::endl;
        }
    }
private:
    void requestRosService(const char *cmd) {
        atank::Command tankCmd;
        tankCmd.request.cmd = std::string(cmd);

        if (client_.call(tankCmd)) {
            if (tankCmd.response.ack == true) {
                ROS_INFO("[CameraRecoder] request service: %s", cmd);
                ROS_INFO("[CameraRecoder] ack: '%s'", tankCmd.response.log.c_str());
            }
            else {
                ROS_INFO("[CameraRecoder] connection is failed.");
            }
        }
        else {
            ROS_INFO("[CameraRecoder] failed to call service.");
        }
    }
    ros::NodeHandle node_;
    ros::ServiceClient client_;
    std::queue<std::string> cmd_queue_;
};

bool videoRecoderEnable = true;

void videoRecoder(const char *recordFileName) {
    cv::VideoCapture capture;
    cv::VideoWriter  videoFrameWriter;

    capture = cv::VideoCapture("http://raspberrypi:8080/stream/video.mjpeg");
    capture.set(CV_CAP_PROP_BUFFERSIZE, 1);


    /* Open video frame, and check the validity
     */
    cv::Mat frame;
    capture >> frame;
    if (frame.rows == 0 || frame.cols == 0) {
        ROS_INFO("[ERROR] Can't connect to the camera.\n");
        return;
    }

    /* Open video frame writer.
     */
    videoFrameWriter.open(recordFileName, cv::VideoWriter::fourcc('M','J','P','G'),\
                20, cv::Size(frame.cols, frame.rows), true);


    /* Write video frame continuously.
     */
    while(videoRecoderEnable) {
        capture >> frame;

        if (frame.channels() == 3) {
            cv::cvtColor(frame, frame, CV_BGR2RGB);
            //cv::cvtColor(frame, frame, CV_BGR2GRAY);
        }

        videoFrameWriter << frame;
    }
 
    return;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_recoder");
    ros::NodeHandle n;

    ROS_INFO("[CameraRecoder] initialization is successful.");

    std::thread *videoRecordThread = new std::thread(videoRecoder, "featureTrackingTest.avi");

    signal(SIGINT, mySignalHandler);

    // "command": topic name
    CommandProcessor cproc(n);

    cproc.pushCommand("version");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");

    cproc.pushCommand("left_turn");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("stop");
    cproc.pushCommand("wait_10ms");

    cproc.pushCommand("right_turn");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("stop");
    cproc.pushCommand("wait_10ms");

    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");
    cproc.pushCommand("wait_10ms");

    while(cproc.size() > 0) {
        cproc.executeCommand();
    }

    videoRecoderEnable = false;
    videoRecordThread->join();

    return 0;
}
