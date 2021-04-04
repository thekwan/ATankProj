#include "ros/ros.h"
#include "atank/Command.h"

struct command_list {
    std::string cmd;
    std::vector<std::string> args;
};

void motor_control(struct command_list & clist);
void uart_control(struct command_list & clist);
void led_control(struct command_list & clist);

struct command_function{
    const char *command;
    void (*func)(struct command_list & clist);
};

struct command_function command_function_list[] = {
    {   "motor", motor_control   },
    {   "uart",  uart_control    },
    {   "led",   led_control     },

    {   nullptr, nullptr         }  // end marker
};

void parse_command(std::string cmd, struct command_list *clist) {
    std::string token;
    std::stringstream ss(cmd);

    int cnt = 0;
    while (std::getline(ss, token, '.')) {
        if (cnt++ == 0) {
            clist->cmd = token;
        }
        else {
            clist->args.push_back(token);
        }
    }
}

bool cmd_proc(atank::Command::Request & req,
              atank::Command::Response & res)
{
    // parse command 
    struct command_list clist;
    parse_command(req.cmd, &clist);

    struct command_function *cf = command_function_list;

    while(cf->command != nullptr) {
        if (clist.cmd.compare(cf->command) == 0) {
            cf->func(clist);
            break;
        }
        cf++;
    }

    if (cf->command == nullptr) {
        ROS_INFO("[ERROR] Unknown command: %s", clist.cmd.c_str());
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

void check_command_list(struct command_list & clist) {
    // DEBUG: print received command string and arguments
    ROS_INFO("[CMD SERVER] get command: '%s'.", clist.cmd.c_str());
    for (int i = 0 ; i < clist.args.size(); i++) {
        ROS_INFO("[CMD SERVER]       args : '%s'.", clist.args[i].c_str());
    }
}

void motor_control(struct command_list & clist) {
    ROS_INFO("[CMD SERVER] 'motor_control' func is called.");
    check_command_list(clist);
}
void uart_control(struct command_list & clist) {
    ROS_INFO("[CMD SERVER] 'uart_control' func is called.");
    check_command_list(clist);
}
void led_control(struct command_list & clist) {
    ROS_INFO("[CMD SERVER] 'led_control' func is called.");
    check_command_list(clist);
}
