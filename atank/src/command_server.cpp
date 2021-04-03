#include "ros/ros.h"
#include "atank/Command.h"

void motor_control(struct command_list *clist);
void uart_control(struct command_list *clist);
void led_control(struct command_list *clist);

struct command_function{
    const char *command;
    void (*func)(struct command_list *);
};

struct command_function command_function_list[] = {
    {   "motor", motor_control   },
    {   "uart",  uart_control    },
    {   "led",   led_control     }
};

struct argument_list {
    std::vector<std::string> args;
};

std::vector<std::string> parse_command(std::string cmd) {
    std::vector<std::string> args;
    std::string token;
    std::stringstream ss(cmd);

    while (std::getline(ss, token, '.')) {
        args.push_back(token);
    }

    return args;
}

bool cmd_proc(atank::Command::Request & req,
              atank::Command::Response & res)
{
    // parse command 
    std::vector<std::string> cmds = parse_command(req.cmd);
    std::string cmd(cmds[0]);

    ROS_INFO("[CMD SERVER] get command: '%s'.", cmd.c_str());
    for (int i = 0 ; i < cmds.size()-1; i++) {
        ROS_INFO("[CMD SERVER]       args : '%s'.", cmds[i+1].c_str());
    }

    std::stringstream ss;
    ss << "'" << req.cmd << "' command is received.";
    res.ack = true;
    res.log = ss.str();

    ROS_INFO("[CMD SERVER] get command '%s'.", req.cmd.c_str());
    ROS_INFO("[CMD SERVER] send ack '%d' and log '%s'.", res.ack, res.log.c_str());

    // memory de-allocation.


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

void motor_control(struct command_list *clist) {
}
void uart_control(struct command_list *clist) {
}
void led_control(struct command_list *clist) {
}
