#include <wx/app.h>
#include <wx/timer.h>

#include <ros/ros.h>

#include <boost/thread.hpp>

#include "SimuFrame.hpp"


namespace arp_master
{

class GraphicsSimuApp : public wxApp
{
public:
  char** local_argv_;
  ros::NodeHandlePtr nh_;

  GraphicsSimuApp()
  {
  }

  bool OnInit()
  {

    // create our own copy of argv, with regular char*s.
    local_argv_ =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
    }

    ros::init(argc, local_argv_, "ARDSimu");
    nh_.reset(new ros::NodeHandle);

    wxInitAllImageHandlers();

    arp_master::SimuFrame* frame = new arp_master::SimuFrame();

    SetTopWindow(frame);
    frame->Show();

    return true;
  }

  int OnExit()
  {
    for ( int i = 0; i < argc; ++i )
    {
      free( local_argv_[ i ] );
    }
    delete [] local_argv_;

    return 0;
  }
};
}

// ne cherchez pas le main() trop loin, il est caché là dedans :
DECLARE_APP(arp_master::GraphicsSimuApp);
IMPLEMENT_APP(arp_master::GraphicsSimuApp);

