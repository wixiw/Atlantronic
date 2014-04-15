/*
 * ActuatorGraphicsNode.cpp
 *
 *  Created on: 11 mars 2014
 *      Author: wla
 */

#include "ActuatorsFrame_2014.hpp"
#include <ros/package.h>
#include <cstdlib>
#include <ctime>


using namespace arp_hml;
using namespace std_msgs;

ActuatorsFrame_2014::ActuatorsFrame_2014():
    wxFrame(NULL,
          wxID_ANY,
          wxT("a"),
          wxPoint(100,500),
          wxSize(length_in_pixel, width_in_pixel),
          wxDEFAULT_FRAME_STYLE)
//,
	//m_updateTimer(this)
{
//	m_updateTimer.Start(100.);
//
//	int hBtnPos = 20;
//	int vBtnPos = 20;
//	int vBtnPosDelta = 40;
//
//	m_leftSuckerTorButton.init(
//	    ID_LEFT_SUCKER_FIRE_TOR_HANDLER,
//	    wxPoint(hBtnPos,vBtnPos),
//	    "Left Sucker Tor",
//	    "Ubiquity2014/leftArmSuckerTor",
//	    this
//	    );
//
//	vBtnPos += vBtnPosDelta;
//    m_rightSuckerTorButton.init(
//        ID_RIGHT_SUCKER_FIRE_TOR_HANDLER,
//        wxPoint(hBtnPos,vBtnPos),
//        "Right Sucker Tor",
//        "Ubiquity2014/rightArmSuckerTor",
//        this
//        );
//
//    vBtnPos += vBtnPosDelta;
//    m_leftScanTorButton.init(
//            ID_LEFT_SUCKER_SCAN_TOR_HANDLER,
//            wxPoint(hBtnPos,vBtnPos),
//            "Left Arm Scan",
//            "Ubiquity2014/leftArmScanTor",
//            this
//        );
//
//    vBtnPos += vBtnPosDelta;
//    m_rightScanTorButton.init(
//            ID_RIGHT_SUCKER_SCAN_TOR_HANDLER,
//            wxPoint(hBtnPos,vBtnPos),
//            "Right Arm Scan",
//            "Ubiquity2014/rightArmScanTor",
//            this
//           );
//
//    vBtnPos += vBtnPosDelta;
//    m_leftArmBlockedButton.init(
//            ID_LEFT_ARM_BLOCKED_HANDLER,
//            wxPoint(hBtnPos,vBtnPos),
//            "Left Arm Blocked",
//            "Ubiquity2014/leftArmBlocked",
//            this
//            );
//
//    vBtnPos += vBtnPosDelta;
//    m_rightArmBlockedButton.init(
//            ID_RIGHT_ARM_BLOCKED_HANDLER,
//            wxPoint(hBtnPos,vBtnPos),
//            "Right Arm Blocked",
//            "Ubiquity2014/rightArmBlocked",
//            this
//            );
//
//	Connect(m_updateTimer.GetId(), wxEVT_TIMER, wxTimerEventHandler(ActuatorsFrame_2014::onUpdate), NULL, this);
//	Connect(wxEVT_PAINT, wxPaintEventHandler(ActuatorsFrame_2014::onPaint), NULL, this);
}

void ActuatorsFrame_2014::onUpdate(wxTimerEvent& evt)
{
  ros::spinOnce();
  Refresh();
  if (!ros::ok())
  {
    Close();
  }
}

void ActuatorsFrame_2014::onPaint(wxPaintEvent& evt)
{
	wxPaintDC dc(this);

    //publication
	m_leftSuckerTorButton.publish();
	m_rightSuckerTorButton.publish();
	m_leftScanTorButton.publish();
	m_rightScanTorButton.publish();
	m_leftArmBlockedButton.publish();
	m_rightArmBlockedButton.publish();
}

/** Mapping C */

void ActuatorsFrame_2014::onLeftSuckerFireEvt(wxCommandEvent& event)
{
    m_leftSuckerTorButton.onButonEvent(event);
}
void ActuatorsFrame_2014::onRightSuckerFireEvt(wxCommandEvent& event)
{
    m_rightSuckerTorButton.onButonEvent(event);
}
void ActuatorsFrame_2014::onLeftArmScanEvt(wxCommandEvent& event)
{
    m_leftScanTorButton.onButonEvent(event);
}
void ActuatorsFrame_2014::onRightArmScanEvt(wxCommandEvent& event)
{
    m_rightScanTorButton.onButonEvent(event);
}
void ActuatorsFrame_2014::onLeftArmBlockedEvt(wxCommandEvent& event)
{
    m_leftArmBlockedButton.onButonEvent(event);
}
void ActuatorsFrame_2014::onRightArmBlockedEvt(wxCommandEvent& event)
{
    m_rightArmBlockedButton.onButonEvent(event);
}
