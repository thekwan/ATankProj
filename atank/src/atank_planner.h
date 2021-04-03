#include <wx/wx.h>
#include "ros/ros.h"

extern ros::NodeHandle *_nh;
extern ros::ServiceClient *_client;

class ATankPlanner {
public:
    ATankPlanner(wxTextCtrl *p_logText)
        : m_logText(p_logText) {}
    ~ATankPlanner() {}

    void keyboard_left_push();
    void keyboard_right_push();
    void keyboard_up_push();
    void keyboard_down_push();
    void keyboard_left_release();
    void keyboard_right_release();
    void keyboard_up_release();
    void keyboard_down_release();
    void reqServiceMove(std::string);

private:
    wxTextCtrl  *m_logText;
};
