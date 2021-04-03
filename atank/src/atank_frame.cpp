#include "atank_frame.h"

#include <map>
#include <vector>
#include <algorithm>

const int wxID_ROS_OPEN = 1;
const int wxID_ROS_CLOSE = 2;
const int wxID_ROS_TEST = 5;
const int wxID_UART_OPEN = 3;
const int wxID_UART_CLOSE = 4;
const int wxID_KEYCTRL_PAD = 6;
const int wxID_CLEAR_LOG = 7;

ros::NodeHandle *_nh = nullptr;
ros::ServiceClient *_client = nullptr;

ATankFrame::ATankFrame(const wxString & title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxDefaultSize),
    _ros_connected(false)
{
    /*
     * Menubar and Menu.
     */
    menuFile = new wxMenu;
    menuFile->Append(wxID_ROS_OPEN,  "ROS &Open\tO",
            "Initializes ROS client program.");
    menuFile->Append(wxID_ROS_CLOSE, "ROS &Close\tC",
            "Deinitializes ROS client program.");
    menuFile->Append(wxID_ROS_TEST,  "ROS &Test\tT",
            "Test ROS connection.");
    menuFile->AppendSeparator();

    menuFile->Append(wxID_CLEAR_LOG, "Clear &Log\tL",
            "Clear log window.");
    menuFile->AppendSeparator();

    menuFile->Append(wxID_KEYCTRL_PAD, "&Key Pad\tK", 
            "Pop-up key pad pannel to control tank.");
    menuFile->AppendSeparator();

    menuFile->Append(wxID_UART_OPEN, "&UART open\tU", 
            "UART connection open to control tank.");
    menuFile->Append(wxID_UART_CLOSE, "U&ART close\tA", 
            "UART connection close.");
    menuFile->AppendSeparator();

    menuFile->Append(wxID_EXIT, wxT("&Quit\tCtrl+Q"));


    // the "Help" menu list
    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT, "&About\tF1", "Show about dialog");

    // now append the freshly created menu to the menu bar...
    menuBar = new wxMenuBar();
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");

    SetMenuBar(menuBar);


    // Log window
    m_logText = new wxTextCtrl(this, wxID_ANY, "",
                               wxDefaultPosition, wxDefaultSize,
                               wxTE_MULTILINE|wxTE_READONLY|wxTE_RICH|wxHSCROLL);

    // set monospace font to have output in nice columns
    wxFont font(10, wxFONTFAMILY_TELETYPE,
                wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    m_logText->SetFont(font);


    // layout
    wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    //sizer->Add(m_inputWin, wxSizerFlags().Expand());
    sizer->Add(m_logText, wxSizerFlags(1).Expand());
    SetSizerAndFit(sizer);

    // set size and position on screen
    SetSize(700, 340);
    CentreOnScreen();


    // connect menu event and handlers
    Connect(wxID_ROS_OPEN, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnRosOpen));
    Connect(wxID_ROS_CLOSE, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnRosClose));
    Connect(wxID_ROS_TEST, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnRosTest));

    Connect(wxID_ABOUT, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnAbout));
    Connect(wxID_CLEAR_LOG, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnClear));
    // Key-Event handler connection.
    Connect(wxID_KEYCTRL_PAD, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnKeyControlPad));

    Connect(wxID_UART_OPEN, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnUartOpen));
    Connect(wxID_UART_CLOSE, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnUartClose));

    Connect(wxID_EXIT, wxEVT_COMMAND_MENU_SELECTED,
            wxCommandEventHandler(ATankFrame::OnQuit));

    // status bar is useful for showing the menu items help strings
    CreateStatusBar();


    /* ROS initialize.
     */
    RosInit();

    /* Tank move planner.
     */
    InitPlanner();

    Show(true);
}

ATankFrame::~ATankFrame()
{
    if (m_keypad_frame) {
        delete m_keypad_frame;
    }
    RosShutdown();
}

void ATankFrame::OnQuit(wxCommandEvent & WXUNUSED(event))
{
    Close(true);
}

void ATankFrame::InitPlanner(void) {
    m_planner = new ATankPlanner(m_logText);
}

void ATankFrame::RosInit(void)
{
    int argc = 1;
    const char *argv[1] = {"command_client"};
    ros::init(argc, (char **)argv, "command_client");
    _nh = new ros::NodeHandle;

    _client = nullptr;
}

void ATankFrame::RosShutdown(void)
{
    ros::shutdown();
    delete _nh;
    if (_ros_connected == true) {
        delete _client;
        _client = nullptr;
    }
}

void ATankFrame::OnRosClose(wxCommandEvent & WXUNUSED(event))
{
    ROS_INFO("[CMD CLIENT] ROS connection is closed.");
    if (_ros_connected) {
        delete _client;
        _client = nullptr;
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
        m_logText->AppendText("Success to open ROS connection.\n");
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

// UART handlers
void ATankFrame::OnUartOpen(wxCommandEvent & WXUNUSED(event))
{
    m_logText->AppendText(wxString("[INFO] OnUartOpen() is called.\n"));
}

void ATankFrame::OnUartClose(wxCommandEvent & WXUNUSED(event))
{
    m_logText->AppendText(wxString("[INFO] OnUartClose() is called.\n"));
}

// event handlers
void ATankFrame::OnAbout(wxCommandEvent& WXUNUSED(event))
{
    wxMessageBox("Atank contoller GUI in wxWidgets\n"
                 "(c) 2020 Deokhwan Kim\n"
                 "(c) 2020 proto19 tech",
                 "About Atank controller",
                 wxOK | wxICON_INFORMATION, this);
}

void ATankFrame::OnKeyControlPad(wxCommandEvent& WXUNUSED(event))
{
    if (m_keypad_frame == nullptr) {
        m_keypad_frame = new KeyPadFrame("Control pannel", m_logText, this);
    }
}


void ATankFrame::key_event_handler(KeyCodeInfo kinfo) {
    wxString msg;

    if (kinfo.state == KeyState::KEY_PUSHED) {
        switch(kinfo.code) {
            case LEFT:  m_planner->keyboard_left_push();  break;
            case RIGHT: m_planner->keyboard_right_push(); break;
            case UP:    m_planner->keyboard_up_push();    break;
            case DOWN:  m_planner->keyboard_down_push();  break;
            default:
                msg.Printf("[WARN] invalid keycode: %5d\n", kinfo.code);
                m_logText->AppendText(msg);
                break;
        }
    }
    else {
        switch(kinfo.code) {
            case LEFT:  m_planner->keyboard_left_release();  break;
            case RIGHT: m_planner->keyboard_right_release(); break;
            case UP:    m_planner->keyboard_up_release();    break;
            case DOWN:  m_planner->keyboard_down_release();  break;
            default:
                msg.Printf("[WARN] invalid keycode: %5d\n", kinfo.code);
                m_logText->AppendText(msg);
                break;
        }
    }
    
}

// helper function that returns textual description of wx virtual keycode
const char* GetVirtualKeyCodeName(int keycode)
{
    switch ( keycode )
    {
#define WXK_(x) \
        case WXK_##x: return #x;

        WXK_(BACK)
        WXK_(TAB)
        WXK_(RETURN)
        WXK_(ESCAPE)
        WXK_(SPACE)
        WXK_(DELETE)
        WXK_(START)
        WXK_(LBUTTON)
        WXK_(RBUTTON)
        WXK_(CANCEL)
        WXK_(MBUTTON)
        WXK_(CLEAR)
        WXK_(SHIFT)
        WXK_(ALT)
        WXK_(CONTROL)
        WXK_(MENU)
        WXK_(PAUSE)
        WXK_(CAPITAL)
        WXK_(END)
        WXK_(HOME)
        WXK_(LEFT)
        WXK_(UP)
        WXK_(RIGHT)
        WXK_(DOWN)
        WXK_(SELECT)
        WXK_(PRINT)
        WXK_(EXECUTE)
        WXK_(SNAPSHOT)
        WXK_(INSERT)
        WXK_(HELP)
        WXK_(NUMPAD0)
        WXK_(NUMPAD1)
        WXK_(NUMPAD2)
        WXK_(NUMPAD3)
        WXK_(NUMPAD4)
        WXK_(NUMPAD5)
        WXK_(NUMPAD6)
        WXK_(NUMPAD7)
        WXK_(NUMPAD8)
        WXK_(NUMPAD9)
        WXK_(MULTIPLY)
        WXK_(ADD)
        WXK_(SEPARATOR)
        WXK_(SUBTRACT)
        WXK_(DECIMAL)
        WXK_(DIVIDE)
        WXK_(F1)
        WXK_(F2)
        WXK_(F3)
        WXK_(F4)
        WXK_(F5)
        WXK_(F6)
        WXK_(F7)
        WXK_(F8)
        WXK_(F9)
        WXK_(F10)
        WXK_(F11)
        WXK_(F12)
        WXK_(F13)
        WXK_(F14)
        WXK_(F15)
        WXK_(F16)
        WXK_(F17)
        WXK_(F18)
        WXK_(F19)
        WXK_(F20)
        WXK_(F21)
        WXK_(F22)
        WXK_(F23)
        WXK_(F24)
        WXK_(NUMLOCK)
        WXK_(SCROLL)
        WXK_(PAGEUP)
        WXK_(PAGEDOWN)
        WXK_(NUMPAD_SPACE)
        WXK_(NUMPAD_TAB)
        WXK_(NUMPAD_ENTER)
        WXK_(NUMPAD_F1)
        WXK_(NUMPAD_F2)
        WXK_(NUMPAD_F3)
        WXK_(NUMPAD_F4)
        WXK_(NUMPAD_HOME)
        WXK_(NUMPAD_LEFT)
        WXK_(NUMPAD_UP)
        WXK_(NUMPAD_RIGHT)
        WXK_(NUMPAD_DOWN)
        WXK_(NUMPAD_PAGEUP)
        WXK_(NUMPAD_PAGEDOWN)
        WXK_(NUMPAD_END)
        WXK_(NUMPAD_BEGIN)
        WXK_(NUMPAD_INSERT)
        WXK_(NUMPAD_DELETE)
        WXK_(NUMPAD_EQUAL)
        WXK_(NUMPAD_MULTIPLY)
        WXK_(NUMPAD_ADD)
        WXK_(NUMPAD_SEPARATOR)
        WXK_(NUMPAD_SUBTRACT)
        WXK_(NUMPAD_DECIMAL)
        WXK_(NUMPAD_DIVIDE)

        WXK_(WINDOWS_LEFT)
        WXK_(WINDOWS_RIGHT)
#ifdef __WXOSX__
        WXK_(RAW_CONTROL)
#endif
#undef WXK_

    default:
        return NULL;
    }
}

// helper function that returns textual description of key in the event
wxString GetKeyName(const wxKeyEvent &event)
{
    int keycode = event.GetKeyCode();
    const char* virt = GetVirtualKeyCodeName(keycode);
    if ( virt )
        return virt;
    if ( keycode > 0 && keycode < 32 )
        return wxString::Format("Ctrl-%c", (unsigned char)('A' + keycode - 1));
    if ( keycode >= 32 && keycode < 128 )
        return wxString::Format("'%c'", (unsigned char)keycode);

#if wxUSE_UNICODE
    int uc = event.GetUnicodeKey();
    if ( uc != WXK_NONE )
        return wxString::Format("'%c'", uc);
#endif

    return "unknown";
}


// Key-pad frame constructor
KeyPadFrame::KeyPadFrame(const wxString& title, wxTextCtrl *p_logText, ATankFrame *parent)
       : wxFrame(NULL, wxID_ANY, title),
         m_inputWin(NULL), m_parent(parent)
{

    m_logText = p_logText;
    m_logText->AppendText("Keyboard control pannel is opened.\n");


    m_inputWin = new wxWindow(this, wxID_ANY,
                              wxDefaultPosition, wxSize(-1, 50),
                              wxRAISED_BORDER);

    m_inputWin->SetBackgroundColour(*wxBLUE);
    m_inputWin->SetForegroundColour(*wxWHITE);


    // connect event handlers for the blue input window
    m_inputWin->Connect(wxEVT_KEY_DOWN, wxKeyEventHandler(KeyPadFrame::OnKeyDown),
                        NULL, this);
    m_inputWin->Connect(wxEVT_KEY_UP, wxKeyEventHandler(KeyPadFrame::OnKeyUp),
                        NULL, this);

    m_inputWin->Connect(wxEVT_PAINT,
                        wxPaintEventHandler(KeyPadFrame::OnPaintInputWin),
                        NULL, this);

    this->Connect(wxEVT_CLOSE_WINDOW,
                        wxCloseEventHandler(KeyPadFrame::OnCloseWindow),
                        NULL, this);

    // layout
    wxBoxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(m_inputWin, wxSizerFlags(1).Expand());
    SetSizerAndFit(sizer);

    // set size and position on screen
    SetSize(300, 200);
    //CentreOnScreen();
    

    // Add valid keycodes.
    _valid_keycode.push_back(KeyCode::LEFT);  // left-arrow.
    _valid_keycode.push_back(KeyCode::RIGHT);  // right-arrow.
    _valid_keycode.push_back(KeyCode::UP);  // up-arrow.
    _valid_keycode.push_back(KeyCode::DOWN);  // down-arrow.

    Show(true);
}

void KeyPadFrame::OnPaintInputWin(wxPaintEvent& WXUNUSED(event))
{
    wxPaintDC dc(m_inputWin);
    dc.SetTextForeground(*wxWHITE);
    wxFont font(*wxSWISS_FONT);
    font.SetWeight(wxFONTWEIGHT_BOLD);
    font.SetPointSize(font.GetPointSize() + 2);
    dc.SetFont(font);

    dc.DrawLabel("Press keys here",
                 m_inputWin->GetClientRect(), wxALIGN_CENTER);
}

void KeyPadFrame::OnCloseWindow(wxCloseEvent & event)
{
    m_logText->AppendText("Keyboard control pannel is closed.\n");
    m_parent->keypad_frame_closed();

    event.Skip();
}

void KeyPadFrame::LogEvent(const wxString& name, wxKeyEvent& event)
{
    wxString msg;
#if 1
    msg.Printf("[%7s] %15s %5d\n", name, GetKeyName(event), event.GetKeyCode());
#else
    // event  key_name  KeyCode  modifiers  Unicode  raw_code raw_flags pos
    msg.Printf("%7s %15s %5d   %c%c%c%c"
#if wxUSE_UNICODE
                   "%5d (U+%04x)"
#else
                   "    none   "
#endif
#ifdef wxHAS_RAW_KEY_CODES
                   "  %7lu    0x%08lx"
#else
                   "  not-set    not-set"
#endif
                   "  (%5d,%5d)"
                   "\n",
               name,
               GetKeyName(event),
               event.GetKeyCode(),
               event.ControlDown() ? 'C' : '-',
               event.AltDown()     ? 'A' : '-',
               event.ShiftDown()   ? 'S' : '-',
               event.MetaDown()    ? 'M' : '-'
#if wxUSE_UNICODE
               , event.GetUnicodeKey()
               , event.GetUnicodeKey()
#endif
#ifdef wxHAS_RAW_KEY_CODES
               , (unsigned long) event.GetRawKeyCode()
               , (unsigned long) event.GetRawKeyFlags()
#endif
               , event.GetX()
               , event.GetY()
               );
#endif
    m_logText->AppendText(msg);
}

bool KeyPadFrame::isInvalidKeyCode(int keycode)
{
    auto it = std::find(_valid_keycode.begin(), _valid_keycode.end(), keycode);
    return (it == _valid_keycode.end());
}

void KeyPadFrame::OnKeyDown(wxKeyEvent& event) {
    int keycode = event.GetKeyCode();
    KeyCodeInfo kinfo;

    // check the valid key event.
    if (isInvalidKeyCode(keycode))
        return;
    
    if (key_state.find(keycode) == key_state.end()
     || key_state[keycode] == KeyState::KEY_RELEASED) {
        LogEvent("KeyDown", event);
        key_state[keycode] = KeyState::KEY_PUSHED;

        // keyboard event process.
        kinfo.code = keycode;
        kinfo.state = KeyState::KEY_PUSHED;
        m_parent->key_event_handler(kinfo);
    }
}

void KeyPadFrame::OnKeyUp(wxKeyEvent& event) {
    int keycode = event.GetKeyCode();
    KeyCodeInfo kinfo;

    // check the valid key event.
    if (isInvalidKeyCode(keycode))
        return;

    if (key_state.find(keycode) == key_state.end()
     || key_state[keycode] == KeyState::KEY_PUSHED) {
        LogEvent("KeyUp", event);
        key_state[keycode] = KeyState::KEY_RELEASED;

        // keyboard event process.
        kinfo.code = keycode;
        kinfo.state = KeyState::KEY_RELEASED;
        m_parent->key_event_handler(kinfo);
    }
}

