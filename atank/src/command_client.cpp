#include <iostream>

#include <wx/wx.h>
#include <wx/timer.h>

#include "ros/ros.h"
#include "atank/Command.h"

#include "atank_frame.h"


class ATankApp : public wxApp {
public:
    virtual bool OnInit(void);
};

bool ATankApp::OnInit(void) {
    ATankFrame *atframe = new ATankFrame(wxT("ATankCtrl"));
    atframe->Show(true);

    return true;
}

IMPLEMENT_APP(ATankApp)
