#include "ros/ros.h"
#include "atank/Command.h"
#include "uart.h"

/*
 * UART configurations.
 */
#define UART0_DEVICE_FILE   "/dev/ttyUSB0"
#define UART0_BAUD_RATE     115200

bool uart0_enabled = false;
UartDriverLite *uart0 = nullptr;

struct command_list {
    std::string cmd;
    std::vector<std::string> args;
};

std::string motor_control(struct command_list & clist);
std::string uart_control(struct command_list & clist);
std::string led_control(struct command_list & clist);

struct command_function{
    const char *command;
    std::string (*func)(struct command_list & clist);
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
    std::string log;
    // parse command 
    struct command_list clist;
    parse_command(req.cmd, &clist);

    struct command_function *cf = command_function_list;

    while(cf->command != nullptr) {
        if (clist.cmd.compare(cf->command) == 0) {
            log += cf->func(clist);
            break;
        }
        cf++;
    }

    if (cf->command == nullptr) {
        ROS_INFO("[ERROR] Unknown command: %s", clist.cmd.c_str());
    }
    

    std::stringstream ss;
    //ss << "'" << req.cmd << "' command is received.\n";
    ss << log;// << "\n";
    res.ack = true;
    res.log = ss.str();

    //ROS_INFO("[CMD SERVER] get command '%s'.", req.cmd.c_str());
    //ROS_INFO("[CMD SERVER] send ack '%d' and log '%s'.", res.ack, res.log.c_str());

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

std::string motor_control(struct command_list & clist) {
    std::string log = "motor_control::";
    ROS_INFO("[CMD SERVER] 'motor_control' func is called.");
    check_command_list(clist);

    log += "TODO (not implemented yet).";

    return log;
}

std::string uart_control(struct command_list & clist) {
    std::string log = "uart_control::";
    ROS_INFO("[CMD SERVER] 'uart_control' func is called.");
    check_command_list(clist);

    if (clist.args.size() < 1) {
        ROS_INFO("[CMD SERVER-UART] error: insufficient command argument.");
        log += "invalid command argument";
        return log;
    }

    // check the argument list
    if (clist.args[0].compare("open") == 0) {
        uart0 = new UartDriverLite(UART0_DEVICE_FILE, UART0_BAUD_RATE);
        if (uart0->isOpened()) {
            log += "UART is opened. ";
        }
        else {
            log += "UART open is failed. ";
        }
    }
    else if (clist.args[0].compare("close") == 0) {
        delete uart0;
        uart0 = nullptr;
        log += "UART is closed. ";
    }

    return log;
}

std::string led_control(struct command_list & clist) {
    std::string log = "led_control::";
    ROS_INFO("[CMD SERVER] 'led_control' func is called.");
    check_command_list(clist);

    log += "TODO (not implemented yet).";

    return log;
}
