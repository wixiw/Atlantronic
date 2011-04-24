/*
 * SimuFrame.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_SIMUFRAME_HPP
#define ARP_MASTER_SIMUFRAME_HPP

#include <wx/wx.h>
#include <wx/event.h>
#include <wx/timer.h>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <arp_master/Spawn.h>
#include <arp_master/Kill.h>
#include <map>

#include "SimuRobot.hpp"


namespace arp_master
{

class SimuFrame : public wxFrame
{

private:
  static const double default_dt_ms = 20.0;
  static const double one_meter_in_pixel = 200.0;
  static const double table_length_in_pixel = 609.0;
  static const double table_width_in_pixel = 429.0;  
public:
  SimuFrame(wxWindow* parent);
  ~SimuFrame();

  void spawnRobot(double x, double y, double angle);

private:
  void onUpdate(wxTimerEvent& evt);
  void onPaint(wxPaintEvent& evt);

  void updateRobot();
  void clear();

  bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  bool respawnCallback(Spawn::Request&, Spawn::Response&);


  ros::NodeHandle nh_;
  ros::ServiceServer clear_srv_;
  ros::ServiceServer respawn_srv_;

  wxTimer* update_timer_;

  wxMemoryDC path_dc_;

  wxImage table_image_;
  wxBitmap table_bitmap_;

  SimuRobotPtr mRobot;
  std::string robot_name;
  wxImage robot_image_;

  ros::WallTime last_robot_update_;

};

}

#endif
