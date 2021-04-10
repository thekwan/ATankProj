#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atank/Command.h"
#include "uart.h"

#include <thread>
#include <signal.h>

/*
 * UART configurations.
 */
#define UART0_DEVICE_FILE   "/dev/ttyUSB0"
#define UART0_BAUD_RATE     115200

bool uart0_enabled = false;
UartDriverLite uart0;

struct command_list {
    std::string cmd;
    std::vector<std::string> args;
};

std::string motor_control(struct command_list & clist);
std::string uart_control(struct command_list & clist);
std::string led_control(struct command_list & clist);

bool cmd_proc(atank::Command::Request & req,
              atank::Command::Response & res);

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

void MessagePublisherThreadWrapper(ros::Publisher *msg_pub) {
    std_msgs::String msg;
    ros::Rate loop_rate(100);

    // message publisher loop
    while(ros::ok()) {
        std::string _msg;

        // check uart is opend.
        if (uart0.isOpened()) {
            uart0.ReceiveMessageUart(_msg);
            msg.data = _msg.c_str();
            msg_pub->publish(msg);
        }

        //ros::spinOnce();
        //loop_rate.sleep();
    }
}

void mySignalHandler(int sig) {
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "command_server");
    ros::NodeHandle n;

    signal(SIGINT, mySignalHandler);

    // "command": topic name
    ros::ServiceServer svr_server = n.advertiseService("command", cmd_proc);
    ROS_INFO("[CMD SERVER] command server is ready to process.");

    ros::Publisher     msg_pub = n.advertise<std_msgs::String> ("uart_msg", 1000);
    std::thread _msg_t0(MessagePublisherThreadWrapper, &msg_pub);
    ROS_INFO("[CMD SERVER] message server is ready to process.");

    ros::spin();

    _msg_t0.join();

    return 0;
}


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

    //log += "TODO (not implemented yet).";

    if (clist.args[0].compare("both") == 0) {
        if (clist.args[1].compare("fw") == 0) {
            uart0.SendMessageUart(std::string("rf"));
            uart0.SendMessageUart(std::string("lsu"));
            uart0.SendMessageUart(std::string("rsu"));
            log += "UART sent message 'rf'+'lsu'+'rsu'";
        }
        if (clist.args[1].compare("stop") == 0) {
            uart0.SendMessageUart(std::string("st"));
            log += "UART sent message 'st'";
        }
    }

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
        uart0.Open(UART0_DEVICE_FILE, UART0_BAUD_RATE);
        if (uart0.isOpened()) {
            log += "UART is opened. ";
        }
        else {
            log += "UART open is failed. ";
        }
    }
    else if (clist.args[0].compare("close") == 0) {
        uart0.Close();
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
