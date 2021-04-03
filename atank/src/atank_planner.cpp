#include "atank_planner.h"
#include "atank/Command.h"

void ATankPlanner::keyboard_left_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.right.fw"));
        m_logText->AppendText("Request 'move' service: left turn.\n");
    }
}

void ATankPlanner::keyboard_right_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.left.fw"));
        m_logText->AppendText("Request 'move' service: right turn.\n");
    }
}

void ATankPlanner::keyboard_up_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.both.fw"));
        m_logText->AppendText("Request 'move' service: go forward.\n");
    }
}

void ATankPlanner::keyboard_down_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.both.bw"));
        m_logText->AppendText("Request 'move' service: go backward.\n");
    }
}

void ATankPlanner::keyboard_left_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.both.stop"));
        m_logText->AppendText("Request 'move' service: stop (left.release).\n");
    }
}

void ATankPlanner::keyboard_right_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.both.stop"));
        m_logText->AppendText("Request 'move' service: stop (right.release).\n");
    }
}

void ATankPlanner::keyboard_up_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.both.stop"));
        m_logText->AppendText("Request 'move' service: stop (up.release).\n");
    }
}

void ATankPlanner::keyboard_down_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        reqServiceMove(std::string("motor.both.stop"));
        m_logText->AppendText("Request 'move' service: stop (down.release).\n");
    }
}

void ATankPlanner::reqServiceMove(std::string msg) {
    atank::Command command;
    command.request.cmd = msg;

    if (_client->call(command)) {
        if (command.response.ack == true) {
            ROS_INFO("[CMD client] Connection is tested and successful.");
        }
        else {
            ROS_INFO("[CMD client] Connection is failed.");
        }
    }
    else {
        ROS_INFO("[CMD CLIENT] Failed to call service.");
    }

    return;
}
