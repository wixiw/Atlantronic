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

#include <arp_core/Pose.h>
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

class GraphicsSimuRobot
{
public:
    /**
     * Constructor
     * \param nh NodeHandle of Graphical simulator
     * \param robot_image wxImage of robot.
     * \param pos Robot initial translation
     * \param orient Robot initial orientation (in radian)
     * \param one_meter_in_pixel number of pixel corresponding to 1 meter in robot_image
     */
    GraphicsSimuRobot(const ros::NodeHandle& nh, const wxImage& robot_image, const Vector2& pos, double orient, double one_meter_in_pixel, std::string topicName);

  /**
   * update one time step and plot trace
   * \param path_dc wx Device Controller (used by the pen to plot trace)
   * \param canvas_width table width in meter
   * \param canvas_height table height in meter
   */
  void update(wxMemoryDC& path_dc, float canvas_width, float canvas_height);

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


  // Callbacks
  /**
   * called every time SimuRobot receives a Pose message
   */
  void poseCallback(const arp_core::PoseConstPtr& c);

  /**
     * called every time SimuRobot receives a SetPen service call
     */
  bool setPenCallback(SetPen::Request&, SetPen::Response&);

  /**
   * local copy of NodeHandle of Physical simulator
   */
  ros::NodeHandle nh_;

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
   * wx Image of Robot.
   */
  wxImage robot_image_;

  /**
   * associated bitmap
   */
  wxBitmap robot_;

  /**
   * boolean used to (des-)activate pen
   */
  bool pen_on_;

  /**
   * pen used to trace route
   */
  wxPen pen_;

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
  ros::Subscriber pose_sub_;


  /**
   * ServiceServer used to (des-)activate pen
   */
  ros::ServiceServer set_pen_srv_;


  /**
   * number of pixels corresponding to 1 meter
   */
  double meter_;

};
typedef boost::shared_ptr<GraphicsSimuRobot> GraphicsSimuRobotPtr;

}

#endif
