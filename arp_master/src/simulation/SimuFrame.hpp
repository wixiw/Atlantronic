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
/** \ingroup arp_master
  * \nonstableyet
  *
  * \class SimuFrame
  *
  * \brief Table Simulation
  *
  * Manage physical and graphical simulation of table and robot
  * Should be divided in two parts
  *
  */

class SimuFrame : public wxFrame
{

private:
/**
 * used to cadence thread for physical simulation.
 * Only for cadencing. The actual time is measured internaly by SimuRobot
 */
  static const double default_dt_ms = 20.0;

  /**
   * one meter in pixel
   */
  static const double one_meter_in_pixel = 200.0;

  /**
   * table length in pixel. Should be the width of table image
   */
  static const double table_length_in_pixel = 609.0;

  /**
   * table width in pixel. Should be the height of table image
   */
  static const double table_width_in_pixel = 429.0;  
public:

  /**
   * Default constructor
   * A timer is set width default_dt_ms time step
   * onUpdate is then called every default_dt_ms
   */
  SimuFrame();
  ~SimuFrame();

  /**
   * used by spawn service
   * \param x desired position in meter along table longest axis
   * \param y desired position in meter along table shortest axis
   * \param angle desired orientation
   */
  void spawnRobot(double x, double y, double angle);

private:
  /**
   * called by timer. Manage physical simulation
   */
  void onUpdate(wxTimerEvent& evt);

  /**
   * called by PaintEvent.
   * Paint table then robot
   */
  void onPaint(wxPaintEvent& evt);

  /**
   * called by onUpdate
   * Measure actual time and call SimuRobot update
   */
  void updateRobot();

  /**
   * clear the table of all trace
   */
  void clear();

  /**
   * used by clear service
   */
  bool clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /**
   * used by respawn service
   */
  bool respawnCallback(Spawn::Request&, Spawn::Response&);

  /**
   * Simulator Nodehandle
   * Instanciated by SimuFrame constructor
   */
  ros::NodeHandle nh_;

  /**
   * ServiceServer used to clear table of all trace
   */
  ros::ServiceServer clear_srv_;

  /**
   * ServiceServer used to respawn robot
   */
  ros::ServiceServer respawn_srv_;

  /**
   * used to call onUpdate
   * Attention, pretty inaccurate
   */
  wxTimer* update_timer_;

  /**
   * DeviceController used to trace route with pen
   */
  wxMemoryDC path_dc_;

  /**
   * table image
   */
  wxImage table_image_;

  /**
   * table associated bitmap
   */
  wxBitmap table_bitmap_;

  /**
   * robot
   */
  SimuRobotPtr mRobot;

  /**
   * robot associated image
   */
  wxImage robot_image_;

  /**
   * used to compute actual time step
   */
  ros::WallTime last_robot_update_;

};

}

#endif
