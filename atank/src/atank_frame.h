#ifndef __ATANK_FRAME_H__
#define __ATANK_FRAME_H__

#include <wx/wx.h>
#include <wx/timer.h>

#include <thread>
//#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "atank/Command.h"
#include "atank/Uart.h"
#include "atank/Spi.h"
#include "atank_planner.h"

enum class KeyState{
    KEY_PUSHED,
    KEY_RELEASED
};

enum KeyCode {
    LEFT = 314,
    RIGHT = 316,
    UP = 315,
    DOWN = 317,
};

typedef struct _KeyCodeInfo {
    int code;
    KeyState  state;
} KeyCodeInfo;

class KeyPadFrame;
//class VideoFrame;
class VideoPanel;

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
    void video_frame_closed(void) {
        //m_video_frame = nullptr;
    }

    void key_event_handler(KeyCodeInfo kinfo);
    void RosUartMsgCallback(const std_msgs::String::ConstPtr & uart);
    void RosSpiMsgCallback(const atank::Spi::ConstPtr & spi);

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
    //static void RosMsgThreadWrapper(ATankFrame *p);

    void OnAbout(wxCommandEvent& event);
    void OnClear(wxCommandEvent& WXUNUSED(event)) { m_logText->Clear(); }
    void OnKeyControlPad(wxCommandEvent& WXUNUSED(event));

    void OnUartOpen(wxCommandEvent & WXUNUSED(event));
    void OnUartClose(wxCommandEvent & WXUNUSED(event));
    void OnSpiOpen(wxCommandEvent & WXUNUSED(event));
    void OnSpiClose(wxCommandEvent & WXUNUSED(event));

    void OnVideoDisplay(wxCommandEvent & WXUNUSED(event));

    /*
     * GUI member variables and functions
     */
    wxMenuBar *menuBar;
    wxMenu    *menuFile;
    wxMenu    *menuHelp;
    
    wxTextCtrl *m_logText;          // log window
    KeyPadFrame *m_keypad_frame;    // key frame handler
    //VideoFrame  *m_video_frame;

    std::thread *_msg_thread;

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

#if 0
class VideoFrame : public wxFrame
{
public:
    VideoFrame(const wxString& title, wxTextCtrl *p_logText, ATankFrame *m_parent);
    ~VideoFrame();

    void OnCloseWindow(wxCloseEvent & event);

private:
    wxWindow   *m_inputWin;
    ATankFrame *m_parent;
    wxTextCtrl *m_logText;
    VideoPanel *m_drawPanel;
};

class VideoPanel : public wxPanel
{
    wxBitmap image;
    wxTimer  timer;
    //cv::VideoCapture capture;
    //cv::Mat frame;
    //cv::Mat  capture;

    std::thread *draw_thread_;
    bool draw_enable_;

public:
    VideoPanel(wxFrame *parent);
    ~VideoPanel(void);

    wxSize getImageSize(void);

    void paintEvent(wxPaintEvent & evt);
    void paintNow();
    void OnTimer(wxTimerEvent & evt);
    void UpdateWindow(void);
    void render(wxDC & dc);
    bool isUpdateOk(void);
};
#endif

#endif  // __ATANK_FRAME_H__
