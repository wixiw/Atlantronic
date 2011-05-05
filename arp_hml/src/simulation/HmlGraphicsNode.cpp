/*
 * HmlGraphicsNode.cpp
 *
 *  Created on: 05 may 2011
 *      Author: wla
 */
#define wxUSE_GUI 1
#include <wx/wx.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "HmlGraphicsFrame.hpp"


namespace arp_hml
{

class HmlGraphicsNode : public wxApp
{
	public:
	char** local_argv_;

	  HmlGraphicsNode()
	  {

	  }

	  ~HmlGraphicsNode()
	  {
	  }

	  bool OnInit()
	  {
		// create our own copy of argv, with regular char*s (because Wx create WxChars instead)
		//how argc and argv are accessed is a mystery
		local_argv_ =  new char*[ argc ];
		for ( int i = 0; i < argc; ++i )
		{
		  local_argv_[ i ] = strdup( wxString( argv[ i ] ).mb_str() );
		}
		ros::init(argc, local_argv_, "HmlSimulation");

		wxInitAllImageHandlers();
		HmlGraphicsFrame* frame = new HmlGraphicsFrame();
		SetTopWindow(frame);
		frame->Show();

		return true;
	  }

	  int OnExit()
	  {

		return 0;
	  }
	};
}

// ne cherchez pas le main() trop loin, il est caché là dedans :
DECLARE_APP(arp_hml::HmlGraphicsNode);

BEGIN_EVENT_TABLE(arp_hml::HmlGraphicsFrame,wxFrame)
	EVT_BUTTON(ID_START_HANDLER, arp_hml::HmlGraphicsFrame::onStart)
	EVT_BUTTON(ID_COLOR_HANDLER, arp_hml::HmlGraphicsFrame::onColor)
	EVT_BUTTON(ID_OBSTACLE_HANDLER, arp_hml::HmlGraphicsFrame::onObstacle)
END_EVENT_TABLE()

IMPLEMENT_APP(arp_hml::HmlGraphicsNode);

