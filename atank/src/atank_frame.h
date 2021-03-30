#include <wx/wx.h>
#include <wx/timer.h>

#include "ros/ros.h"
#include "atank/Command.h"

class KeyPadFrame;
class ATankFrame : public wxFrame
{
public:
    ATankFrame(const wxString & title);
    ~ATankFrame();

    /*
     * Keypad frame member funcs.
     */
    void keypad_frame_closed(void) {
        m_keypad_frame = nullptr;
    }

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

    void OnAbout(wxCommandEvent& event);
    void OnClear(wxCommandEvent& WXUNUSED(event)) { m_logText->Clear(); }
    void OnKeyControlPad(wxCommandEvent& WXUNUSED(event));

    /*
     * GUI member variables and functions
     */
    wxMenuBar *menuBar;
    wxMenu    *menuFile;
    wxMenu    *menuHelp;
    
    wxTextCtrl *m_logText;          // log window
    KeyPadFrame *m_keypad_frame;    // key frame handler

    void OnQuit(wxCommandEvent & WXUNUSED(event));
};

class KeyPadFrame : public wxFrame
{
public:
    KeyPadFrame(const wxString& title, wxTextCtrl *p_logText, ATankFrame *m_parent);

private:
    void OnKeyDown(wxKeyEvent& event);
    void OnKeyUp(wxKeyEvent& event);

    void OnCloseWindow(wxCloseEvent & event);

    void LogEvent(const wxString& name, wxKeyEvent& event);
    bool isInvalidKeyCode(int keycode);

    // event handlers
    void OnPaintInputWin(wxPaintEvent& event);

    wxWindow   *m_inputWin;
    ATankFrame *m_parent;
    wxTextCtrl *m_logText;

    enum class KeyState{
        KEY_PUSHED,
        KEY_RELEASED
    };
    std::map<int, KeyState> key_state;
    std::vector<int> _valid_keycode;
};
