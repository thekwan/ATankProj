#ifndef __ATANK_FRAME_H__
#define __ATANK_FRAME_H__

#include <wx/wx.h>
#include <wx/timer.h>

#include "ros/ros.h"
#include "atank/Command.h"
#include "atank_planner.h"

enum class KeyState{
    KEY_PUSHED,
    KEY_RELEASED
};

enum KeyCode {
    LEFT = 314,
    RIGHT = 315,
    UP = 316,
    DOWN = 317,
};

typedef struct _KeyCodeInfo {
    int code;
    KeyState  state;
} KeyCodeInfo;

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

    void key_event_handler(KeyCodeInfo kinfo);

private:
    /* 
     * ROS member variables and functions.
     */
    bool _ros_connected;
    void RosInit(void);
    void RosShutdown(void);
    void OnRosOpen(wxCommandEvent & WXUNUSED(event));
    void OnRosClose(wxCommandEvent & WXUNUSED(event));
    void OnRosTest(wxCommandEvent & WXUNUSED(event));

    void OnAbout(wxCommandEvent& event);
    void OnClear(wxCommandEvent& WXUNUSED(event)) { m_logText->Clear(); }
    void OnKeyControlPad(wxCommandEvent& WXUNUSED(event));

    void OnUartOpen(wxCommandEvent & WXUNUSED(event));
    void OnUartClose(wxCommandEvent & WXUNUSED(event));

    /*
     * GUI member variables and functions
     */
    wxMenuBar *menuBar;
    wxMenu    *menuFile;
    wxMenu    *menuHelp;
    
    wxTextCtrl *m_logText;          // log window
    KeyPadFrame *m_keypad_frame;    // key frame handler

    /*
     * Tank move planer.
     */
    void InitPlanner(void);
    ATankPlanner *m_planner;

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

    std::map<int, KeyState> key_state;
    std::vector<KeyCode> _valid_keycode;
};

#endif  // __ATANK_FRAME_H__
