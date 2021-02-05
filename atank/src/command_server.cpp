#include "ros/ros.h"
#include "atank/Command.h"

bool cmd_proc(atank::Command::Request & req,
              atank::Command::Response & res)
{
    std::stringstream ss;
    ss << "'" << req.cmd << "' command is received.";
    res.ack = true;
    res.log = ss.str();

    ROS_INFO("[CMD SERVER] get command '%s'.", req.cmd.c_str());
    ROS_INFO("[CMD SERVER] send ack '%d' and log '%s'.", res.ack, res.log.c_str());

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "command_server");
    ros::NodeHandle n;

    // "command": topic name
    ros::ServiceServer server = n.advertiseService("command", cmd_proc);
    ROS_INFO("[CMD SERVER] command server is ready to process.");
    ros::spin();

    return 0;
}
