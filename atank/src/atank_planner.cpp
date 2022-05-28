#include "atank_planner.h"
#include "atank/Command.h"

void ATankPlanner::keyboard_left_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.right.fw"));
        reqServiceMove(std::string("uart.lt"));     // set direction
        //reqServiceMove(std::string("uart.bsu"));    // set speed (old method)
        reqServiceMove(std::string("uart.set_speed_30"));    // set speed
        m_logText->AppendText("Request 'move' service: left turn.\n");
    }
}

void ATankPlanner::keyboard_right_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.left.fw"));
        reqServiceMove(std::string("uart.rt"));     // set direction
        //reqServiceMove(std::string("uart.bsu"));    // set speed (old method)
        reqServiceMove(std::string("uart.set_speed_30"));    // set speed
        m_logText->AppendText("Request 'move' service: right turn.\n");
    }
}

void ATankPlanner::keyboard_up_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.both.fw"));
        reqServiceMove(std::string("uart.rf"));
        //reqServiceMove(std::string("uart.bsu"));
        reqServiceMove(std::string("uart.set_speed_30"));    // set speed
        m_logText->AppendText("Request 'move' service: go forward.\n");
    }
}

void ATankPlanner::keyboard_down_push() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.both.bw"));
        reqServiceMove(std::string("uart.rb"));
        //reqServiceMove(std::string("uart.bsu"));
        reqServiceMove(std::string("uart.set_speed_30"));    // set speed
        m_logText->AppendText("Request 'move' service: go backward.\n");
    }
}

void ATankPlanner::keyboard_left_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.both.stop"));
        reqServiceMove(std::string("uart.st"));
        m_logText->AppendText("Request 'move' service: stop (left.release).\n");
    }
}

void ATankPlanner::keyboard_right_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.both.stop"));
        reqServiceMove(std::string("uart.st"));
        m_logText->AppendText("Request 'move' service: stop (right.release).\n");
    }
}

void ATankPlanner::keyboard_up_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.both.stop"));
        reqServiceMove(std::string("uart.st"));
        m_logText->AppendText("Request 'move' service: stop (up.release).\n");
    }
}

void ATankPlanner::keyboard_down_release() {
    ros::ServiceClient *_mvHandle = _client;

    if (_mvHandle) {
        //reqServiceMove(std::string("uart.both.stop"));
        reqServiceMove(std::string("uart.st"));
        m_logText->AppendText("Request 'move' service: stop (down.release).\n");
    }
}

void ATankPlanner::reqServiceMove(std::string msg) {
    atank::Command command;
    command.request.cmd = msg;

    if (_client->call(command)) {
        if (command.response.ack == true) {
            ROS_INFO("[CMD client] Connection is tested and successful.");
            ROS_INFO("   - feedback log: %s", command.response.log.c_str());
        }
        else {
            ROS_INFO("[CMD client] Connection is failed.");
            ROS_INFO("   - feedback log: %s", command.response.log.c_str());
        }
    }
    else {
        ROS_INFO("[CMD CLIENT] Failed to call service.");
    }

    return;
}
