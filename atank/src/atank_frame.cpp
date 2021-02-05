#include "atank_frame.h"

const int wxID_ROS_OPEN = 1;
const int wxID_ROS_CLOSE = 2;
const int wxID_ROS_TEST = 5;
const int wxID_UART_OPEN = 3;
const int wxID_UART_CLOSE = 4;

ATankFrame::ATankFrame(const wxString & title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxDefaultSize),
    _ros_connected(false)
{
    /*
     * Menubar and Menu.
     */
    menubar = new wxMenuBar;
    file = new wxMenu;
    file->Append(wxID_ROS_OPEN, wxT("ROS &Open"));
    file->Append(wxID_ROS_CLOSE, wxT("ROS &Close"));
    file->Append(wxID_ROS_TEST, wxT("ROS &Test"));
    file->AppendSeparator();

    file->Append(wxID_UART_OPEN, wxT("UART &Connect"));
    file->Append(wxID_UART_OPEN, wxT("UART &Disconnect"));
    file->AppendSeparator();

    file->Append(wxID_EXIT, wxT("&Quit\tCtrl+Q"));

    menubar->Append(file, wxT("&File"));

    Connect(wxID_ROS_OPEN, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnRosOpen));
    Connect(wxID_ROS_CLOSE, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnRosClose));
    Connect(wxID_ROS_TEST, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnRosTest));

    Connect(wxID_EXIT, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnQuit));

    SetMenuBar(menubar);

    /* ROS initialize.
     */
    RosInit();

    Centre();
}

ATankFrame::~ATankFrame()
{
    RosShutdown();
}

void ATankFrame::OnQuit(wxCommandEvent & WXUNUSED(event))
{
    Close(true);
}


void ATankFrame::RosInit(void)
{
    int argc = 1;
    const char *argv[1] = {"command_client"};
    ros::init(argc, (char **)argv, "command_client");
    _nh = new ros::NodeHandle;
}

void ATankFrame::RosShutdown(void)
{
    ros::shutdown();
    delete _nh;
    if (_ros_connected == true) {
        delete _client;
    }
}

void ATankFrame::OnRosClose(wxCommandEvent & WXUNUSED(event))
{
    ROS_INFO("[CMD CLIENT] ROS connection is closed.");
    if (_ros_connected) {
        delete _client;
    }
    _ros_connected = false;
}

void ATankFrame::OnRosOpen(wxCommandEvent & WXUNUSED(event))
{
    if (_ros_connected == false) {
        std::string topic_name = "command";
        _client = new ros::ServiceClient;
        *_client = _nh->serviceClient<atank::Command>(topic_name.c_str());

        ROS_INFO("[CMD client] Service instance is initialized.");
        _ros_connected = true;
    }
}

void ATankFrame::OnRosTest(wxCommandEvent & WXUNUSED(event))
{
    if (_ros_connected == true) {
        // Test the connection.
        atank::Command command;
        std::string test_msg("TEST_MESSAGE");
        command.request.cmd = test_msg;

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
    }
    else {
        ROS_INFO("[CMD CLIENT] there is no connection.");
    }
}
