/*
 * SimuRobot.hpp
 *
 *  Created on: 15 apr. 2011
 *      Author: boris
 */

#ifndef ARP_MASTER_SIMUROBOT_HPP
#define ARP_MASTER_SIMUROBOT_HPP

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <wx/wx.h>

#include <math/Geometry.hpp>
#include <math/math.hpp>

//ros messages
#include <arp_core/Pose.h>
#include <arp_core/Velocity.h>
#include <arp_core/DifferentialCommand.h>
#include <arp_core/Odo.h>
//ros services
#include <arp_master/SetPen.h>

using namespace arp_math;
using namespace arp_core;
using namespace arp_master;

namespace arp_master
{

class SimuRobot
{
public:
  SimuRobot(const ros::NodeHandle& nh, const wxImage& robot_image, const Vector2& pos, double orient, double one_meter_in_pixel);

  void update(double dt, wxMemoryDC& path_dc, wxColour background_color, float canvas_width, float canvas_height);
  void paint(wxDC& dc);


private:
  static const int DEFAULT_PEN_R = 0xb3;
  static const int DEFAULT_PEN_G = 0xb8;
  static const int DEFAULT_PEN_B = 0xff;


  // Callbacks
  void commandCallback(const DifferentialCommandConstPtr& c);
  bool setPenCallback(SetPen::Request&, SetPen::Response&);

  ros::NodeHandle nh_;

  // Internal stuff
  wxImage robot_image_;
  wxBitmap robot_;

  Vector2 pos_;
  Rotation2 orient_;
  Vector2 old_pos_;
  Rotation2 old_orient_;

  double v_left_;
  double v_right_;
  double lin_vel_;
  double ang_vel_;
  bool pen_on_;
  wxPen pen_;

  double odo_left_;
  double odo_right_;

  int canvas_x_;
  int canvas_y_;
  int old_canvas_x_;
  int old_canvas_y_;


  // Suscribers
  ros::Subscriber differential_command_sub_;

  // Publishers
  ros::Publisher pose_pub_;
  ros::Publisher odo_pub_;

  // Services
  ros::ServiceServer set_pen_srv_;

  ros::WallTime last_command_time_;

  double meter_;

};
typedef boost::shared_ptr<SimuRobot> SimuRobotPtr;

}

#endif
