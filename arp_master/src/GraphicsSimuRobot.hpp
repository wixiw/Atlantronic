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

#include "math/Geometry.hpp"

#include <arp_core/Pose.h>
#include <arp_core/Velocity.h>
#include <arp_core/DifferentialCommand.h>
#include <arp_core/Odo.h>

#include <arp_master/SetPen.h>


#include <wx/wx.h>

#include "math/math.hpp"


using namespace arp_math;
using namespace arp_master;

namespace arp_master
{
/** \ingroup arp_master
* \nonstableyet
*
* \class SimuRobot
*
* \brief Robot Simulation
*
* Use differential command to simulate robot deplacement on the table.
* Emulate odo data.
*
*/

class SimuRobot
{
public:
    /**
     * Constructor
     * \param nh NodeHandle of Physical simulator
     * \param robot_image wxImage of robot.
     * \param pos Robot initial translation
     * \param orient Robot initial orientation (in radian)
     * \param one_meter_in_pixel number of pixel corresponding to 1 meter in robot_image
     */
  SimuRobot(const ros::NodeHandle& nh, const wxImage& robot_image, const Vector2& pos, double orient, double one_meter_in_pixel);

  /**
   * update one time step and plot trace
   * \param dt time step in sec
   * \param path_dc wx Device Controller (used by the pen to plot trace)
   * \param canvas_width table width in meter
   * \param canvas_height table height in meter
   */
  void update(double dt, wxMemoryDC& path_dc, float canvas_width, float canvas_height);

  /**
   * plot robot
   * \param dc wx Device Controller for pen
   */
  void paint(wxDC& dc);


private:
  /**
   * Default color for pen (trace)
   */
  static const int DEFAULT_PEN_R = 0xb3;
  static const int DEFAULT_PEN_G = 0xb8;
  static const int DEFAULT_PEN_B = 0xff;

  /**
   * distance between wheels in meter
   */
  double base_line;

  /**
   * wheel diameter in meter
   */
  double wheel_diameter;


  // Callbacks
  /**
   * called every time SimuRobot receives a DifferentialCommand message
   */
  void commandCallback(const arp_core::DifferentialCommandConstPtr& c);

  /**
     * called every time SimuRobot receives a SetPen service call
     */
  bool setPenCallback(SetPen::Request&, SetPen::Response&);

  /**
   * local copy of NodeHandle of Physical simulator
   */
  ros::NodeHandle nh_;

  /**
   * wx Image of Robot.
   */
  wxImage robot_image_;

  /**
   * associated bitmap
   */
  wxBitmap robot_;

  /**
   * current position (translation)
   */
  Vector2 pos_;

  /**
   * current orientation
   */
  Rotation2 orient_;

  /**
   * last position (translation)
   */
  Vector2 old_pos_;

  /**
   * last orientation
   */
  Rotation2 old_orient_;

  /**
   * desired angular velocity (in rad/sec) for left wheel (received via DifferentialMessage)
   */
  double v_left_;

  /**
   * desired angular velocity (in rad/sec) for right wheel (received via DifferentialMessage)
   */
  double v_right_;

  /**
   * linear velocity (in m/sec) computed from v_left_ and v_right_
   */
  double lin_vel_;

  /**
   * angular velocity (in rad/sec) computed from v_left_ and v_right_
   */
  double ang_vel_;

  /**
   * boolean used to (des-)activate pen
   */
  bool pen_on_;

  /**
   * pen used to trace route
   */
  wxPen pen_;

  /**
   * left odo data (in radian, cumulated since start)
   */
  double odo_left_;

  /**
   * right odo data (in radian, cumulated since start)
   */
  double odo_right_;

  /**
   * current position in pixel along longest axis
   */
  int canvas_x_;

  /**
   * current position in pixel along short axis
   */
  int canvas_y_;

  /**
   * last position in pixel along longest axis
   */
  int old_canvas_x_;

  /**
   * last position in pixel along shortest axis
   */
  int old_canvas_y_;


  /**
   * Subscriber used to receive DifferentialCommand
   */
  ros::Subscriber differential_command_sub_;

  /**
   * Publisher used to publish Pose
   */
  ros::Publisher pose_pub_;

  /**
   * Publisher used to publish Odo
   */
  ros::Publisher odo_pub_;


  /**
   * ServiceServer used to (des-)activate pen
   */
  ros::ServiceServer set_pen_srv_;

  /**
   * time of last command received
   */
  ros::WallTime last_command_time_;

  /**
   * number of pixels corresponding to 1 meter
   */
  double meter_;

};
typedef boost::shared_ptr<SimuRobot> SimuRobotPtr;

}

#endif
