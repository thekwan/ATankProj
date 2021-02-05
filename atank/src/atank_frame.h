#include <wx/wx.h>
#include <wx/timer.h>

#include "ros/ros.h"
#include "atank/Command.h"

class ATankFrame : public wxFrame
{
public:
    ATankFrame(const wxString & title);
    ~ATankFrame();

private:
    /* 
     * ROS member variables and functions.
     */
    ros::NodeHandle *_nh;
    ros::ServiceClient *_client;
    bool _ros_connected;
    void RosInit(void);
    void RosShutdown(void);
    void OnRosOpen(wxCommandEvent & WXUNUSED(event));
    void OnRosClose(wxCommandEvent & WXUNUSED(event));
    void OnRosTest(wxCommandEvent & WXUNUSED(event));

    /*
     * GUI member variables and functions
     */
    wxMenuBar *menubar;
    wxMenu    *file;
    //wxMenu    *tools;
    void OnQuit(wxCommandEvent & WXUNUSED(event));
};
