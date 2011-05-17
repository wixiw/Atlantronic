/*
 * HmlGraphicsFrame.cpp
 *
 *  Created on: 05 may 2011
 *      Author: wla
 */

#include "HmlGraphicsFrame.hpp"
#include <ros/package.h>
#include <cstdlib>
#include <ctime>


using namespace arp_hml;
using namespace arp_core;
using namespace std_msgs;

HmlGraphicsFrame::HmlGraphicsFrame():
	m_nodeHandle(),
	color_pub(),
	start_pub(),
	obstacle_pub(),
	wxFrame(NULL,
          wxID_ANY,
          wxT("Hml Simulation"),
          wxDefaultPosition, 
          wxSize(length_in_pixel, width_in_pixel),
          wxDEFAULT_FRAME_STYLE)
{
	m_color.color = "red";
	m_start.go = true;
	m_obstacle.detected = false;
	m_emergency.data = false;

	update_timer_ = new wxTimer(this);
	update_timer_->Start(20.);

	m_startButton = new wxButton(this, ID_START_HANDLER, wxT("Start"), wxPoint(20,60),wxDefaultSize);
	m_colorButton = new wxButton(this, ID_COLOR_HANDLER, wxT("Color"), wxPoint(20,20),wxDefaultSize);
	m_obstacleButton = new wxButton(this, ID_OBSTACLE_HANDLER, wxT("Obstacle"), wxPoint(20,100),wxDefaultSize);
	m_emergencyButton = new wxButton(this, ID_EMERGENCY_HANDLER, wxT("AU"), wxPoint(20,140),wxDefaultSize);

	Connect(update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler(HmlGraphicsFrame::onUpdate), NULL, this);
	Connect(wxEVT_PAINT, wxPaintEventHandler(HmlGraphicsFrame::onPaint), NULL, this);

	start_pub = m_nodeHandle.advertise<Start>("Protokrot/start", 1);
	color_pub = m_nodeHandle.advertise<StartColor>("Protokrot/color", 1);
	obstacle_pub = m_nodeHandle.advertise<Obstacle>("/obstacle", 1);
	emergency_pub = m_nodeHandle.advertise<Bool>("Protokrot/emergency_stop", 1);

	ROS_INFO("Starting HmlGraphics with node name %s", ros::this_node::getName().c_str()) ;
}

HmlGraphicsFrame::~HmlGraphicsFrame()
{
	  delete m_startButton ;
	  delete m_colorButton ;
	  delete m_obstacleButton ;
	  delete m_emergencyButton;
	  delete update_timer_ ;
}

void HmlGraphicsFrame::onUpdate(wxTimerEvent& evt)
{
  ros::spinOnce();
  Refresh();
  if (!ros::ok())
  {
    Close();
  }
}

void HmlGraphicsFrame::onPaint(wxPaintEvent& evt)
{
	double distance;
	wxPaintDC dc(this);

    //dessin du rectangle pour la couleur
    if( m_color.color == "red" )
    {
    	dc.SetBrush(*wxRED_BRUSH) ;
    }
    else
    {
    	dc.SetBrush(*wxBLUE_BRUSH) ;
    }
    dc.DrawRectangle(125,20,180,30);

	//dessin du start
    if( !m_start.go )
    {
    	dc.SetBrush(*wxBLACK_BRUSH);
    	distance = 0;
    }
    else
    {
    	dc.SetBrush(*wxGREY_BRUSH);
    	distance = 50;
    }
    dc.DrawCircle(140,75,15);
    dc.SetBrush(*wxBLACK_BRUSH);
    dc.DrawRectangle(150+distance,70,50,10);

    //dessin de l'obstacle
    if( m_obstacle.detected )
    {
    	dc.DrawLine(140,100,200,130);
    	dc.DrawLine(200,100,140,130);
    	dc.DrawLine(180,100,240,130);
    	dc.DrawLine(240,100,180,130);
    	dc.DrawLine(220,100,280,130);
    	dc.DrawLine(280,100,220,130);
    }

    //dessin de l'AU
    if( m_emergency.data )
    {
        dc.SetBrush(*wxRED_BRUSH);
    }
    else
    {
        dc.SetBrush(*wxGREEN_BRUSH);
    }
    dc.DrawCircle(180,155,15);
    dc.SetBrush(*wxBLACK_BRUSH);


	//publication
	start_pub.publish(m_start);
	color_pub.publish(m_color);
	obstacle_pub.publish(m_obstacle);
	emergency_pub.publish(m_emergency);
}

void HmlGraphicsFrame::onStart(wxCommandEvent& event)
{
	//inversion du bouton
	m_start.go = !m_start.go;
}

void HmlGraphicsFrame::onColor(wxCommandEvent& event)
{
	//inversion de la couleur
	if( m_color.color == "red" )
		m_color.color = "blue";
	else
		m_color.color = "red";
}

void HmlGraphicsFrame::onObstacle(wxCommandEvent& event)
{
	//inversion du bouton
	m_obstacle.detected = !m_obstacle.detected;
}

void HmlGraphicsFrame::onEmergency(wxCommandEvent& event)
{
    //inversion du bouton
    m_emergency.data = !m_emergency.data;
}
