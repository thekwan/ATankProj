#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atank/Command.h"
#include "atank/Spi.h"
#include "uart.h"
#include "spi.h"

#include <thread>
#include <signal.h>

/*
 * UART configurations.
 */
//#define UART0_DEVICE_FILE   "/dev/ttyUSB0"
#define UART0_DEVICE_FILE   "/dev/ttyS0"
#define UART0_BAUD_RATE     115200

#define SPI0_DEVICE_FILE   "/dev/spidev0.0"
#define SPI0_SPEED_HZ      100000  // 0.1MHz

bool uart0_enabled = false;
UartDriverLite uart0;
SpiDriverLite spi0;

struct command_list {
    std::string cmd;
    std::vector<std::string> args;
};

std::string motor_control(struct command_list & clist);
std::string uart_control(struct command_list & clist);
std::string spi_control(struct command_list & clist);
std::string led_control(struct command_list & clist);
std::string version(struct command_list & clist);

bool cmd_proc(atank::Command::Request & req,
              atank::Command::Response & res);

struct command_function{
    const char *command;
    std::string (*func)(struct command_list & clist);
};

struct command_function command_function_list[] = {
    {   "motor", motor_control   },
    //{   "uart",  uart_control    },
    {   "spi",   spi_control     },
    {   "led",   led_control     },
    {   "version",   version     },

    {   nullptr, nullptr         }  // end marker
};

#if 0
void MessagePublisherThreadWrapperUart(ros::Publisher *msg_pub) {
    std_msgs::String msg;
    ros::Rate loop_rate(10);

    // message publisher loop
    while(ros::ok()) {
        std::string _msg;

        // check uart is opend.
        if (uart0.isOpened()) {
            uart0.ReceiveMessageUart(_msg);
            msg.data = _msg.c_str();
            msg_pub->publish(msg);
            //ROS_INFO("Rx %s", _msg.c_str());
        }

        ros::spinOnce();	// check it once again.
        loop_rate.sleep();	// check it once again.
    }
}
#endif

char tx[2], rx[1024];

void MessagePublisherThreadWrapperSpi(ros::Publisher *msg_pub) {
    atank::Spi msg;
    ros::Rate loop_rate(10);
    int i;

    // message publisher loop
    while(ros::ok()) {
        //std::string _msg;

        // check uart is opend.
        if (spi0.isOpened()) {
            int rx_len;
            int data_size;

            tx[0] = 0xD;
            rx[0] = 0x0;
            rx[1] = 0x0;

            rx_len = spi0.SendAndReceiveBytes(tx, 1, rx, 2);

            data_size = (((int)rx[1]) << 8) | ((int)rx[0]);
            ROS_INFO("SpiDataSize = %d", data_size);

            if (data_size > 1024) {
                data_size = 1024;
                ROS_INFO("Warning... data size is over '1024'");
            }

            rx_len = spi0.SendAndReceiveBytes(tx, 0, rx, data_size);

            msg.data_size = (uint16_t) data_size;
            for(i = 0; i < data_size; i++) {
                msg.data.push_back(rx[i]);
            }
            msg_pub->publish(msg);
        }

        ros::spinOnce();	// check it once again.
        loop_rate.sleep();	// check it once again.
    }
}

void mySignalHandler(int sig) {
    ROS_INFO("Signal handler is called. Server will be terminated.");
    ros::shutdown();
    uart0.Close();
    spi0.Close();
    exit(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "command_server");
    ros::NodeHandle n;

    // UART open.
    uart0.Open(UART0_DEVICE_FILE, UART0_BAUD_RATE);

    signal(SIGINT, mySignalHandler);

    // "command": topic name
    ros::ServiceServer svr_server = n.advertiseService("command", cmd_proc);
    ROS_INFO("[CMD SERVER] command server is ready to process.");

    //ros::Publisher     uart_msg_pub = n.advertise<std_msgs::String> ("uart_msg", 1000);
    //std::thread _msg_t0(MessagePublisherThreadWrapperUart, &uart_msg_pub);

    ros::Publisher     spi_msg_pub = n.advertise<atank::Spi> ("spi_msg", 1000);
    std::thread _msg_t1(MessagePublisherThreadWrapperSpi, &spi_msg_pub);

    ROS_INFO("[CMD SERVER] message server is ready to process.");

    ros::spin();

    //_msg_t0.join();
    _msg_t1.join();

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
    std::string ack_;

    if (clist.args[0].compare("both") == 0) {
        if (clist.args[1].compare("fw") == 0) {
            uart0.SendMessageUart(std::string("rf"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
            uart0.SendMessageUart(std::string("bsu"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
        }
        else if (clist.args[1].compare("bw") == 0) {
            uart0.SendMessageUart(std::string("rb"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
            uart0.SendMessageUart(std::string("bsu"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
        }
        else if (clist.args[1].compare("stop") == 0) {
            uart0.SendMessageUart(std::string("st"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
        }
        else {
            log += "Unknown UART arg[1].";
        }
    }
    else if (clist.args[0].compare("right") == 0) {
        if (clist.args[1].compare("fw") == 0) {
            uart0.SendMessageUart(std::string("lt"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
            uart0.SendMessageUart(std::string("bsu"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
        }
        else {
            log += "Unknown UART arg[1].";
        }
    }
    else if (clist.args[0].compare("left") == 0) {
        if (clist.args[1].compare("fw") == 0) {
            uart0.SendMessageUart(std::string("rt"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
            uart0.SendMessageUart(std::string("bsu"));
            uart0.ReceiveMessageUart(ack_);
            log += ack_;
        }
        else {
            log += "Unknown UART arg[1].";
        }
    }
    else {
        log += "Unknown UART arg[0].";
    }

    return log;
}

#if 0
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
#endif

std::string spi_control(struct command_list & clist) {
    std::string log = "spi_control::";
    ROS_INFO("[CMD SERVER] 'spi_control' func is called.");
    check_command_list(clist);

    if (clist.args.size() < 1) {
        ROS_INFO("[CMD SERVER-UART] error: insufficient command argument.");
        log += "invalid command argument";
        return log;
    }

    // check the argument list
    if (clist.args[0].compare("open") == 0) {
        spi0.Open(SPI0_DEVICE_FILE, SPI0_SPEED_HZ);
        if (spi0.isOpened()) {
            log += "SPI is opened. ";
        }
        else {
            log += "SPI open is failed. ";
        }
    }
    else if (clist.args[0].compare("close") == 0) {
        spi0.Close();
        log += "SPI is closed. ";
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

std::string version(struct command_list & clist) {
    std::string log = "version::";
    ROS_INFO("[CMD SERVER] 'version' func is called.");
    check_command_list(clist);

    // send command message
    uart0.SendMessageUart(std::string("version"));

    // receive ack message
    std::string _msg("No response from UART");
    uart0.ReceiveMessageUart(_msg);

    return log + _msg;
}
