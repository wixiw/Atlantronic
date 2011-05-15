/*
 * HmlGraphicsFrame.hpp
 *
 *  Created on: 05 may 2011
 *      Author: wla
 */

#ifndef ARP_HML_SIMUFRAME_HPP
#define ARP_HML_SIMUFRAME_HPP

#define wxUSE_GUI 1
#include <wx/wx.h>
#include <ros/ros.h>

#include <arp_core/Obstacle.h>
#include <arp_core/StartColor.h>
#include <arp_core/Start.h>

#define ID_START_HANDLER 1
#define ID_COLOR_HANDLER 2
#define ID_OBSTACLE_HANDLER 3

using namespace arp_core;

namespace arp_hml
{
/** \ingroup arp_hml
  * \nonstableyet
  *
  * \class HmlGraphicsFrame
  *
  * \brief Hml Simulation IHM
  *
  * Allow user to interact as if it was playing with robot hardware
  *
  */

class HmlGraphicsFrame : public wxFrame
{
	public:

	  /**
	   * Default constructor
	   * A timer is set width default_dt_ms time step
	   * onUpdate is then called every default_dt_ms
	   */
		HmlGraphicsFrame();
	  ~HmlGraphicsFrame();


	protected:

	  /**
	   * Simulator Nodehandle
	   * Instanciated by GraphicsSimuFrame constructor
	   */
	  ros::NodeHandle m_nodeHandle;

	  /**
	   * table length in pixel. Should be the width of table image
	   */
	  static const double length_in_pixel = 400.0;

	  /**
	   * table width in pixel. Should be the height of table image
	   */
	  static const double width_in_pixel = 200.0;

	  /**
	   * Bouton représentant le start
	   */
	  wxButton* m_startButton;

	  /**
	   * Bouton représentant la couleur
	   */
	  wxButton* m_colorButton;

	  /**
	   * Button représentant les obstacles
	   */
	  wxButton* m_obstacleButton;

	  /**
	   * Représente l'état du switch de la couleur
	   */
	  StartColor m_color;

	  /**
	   * Représente l'état du switch du start
	   */
	  Start m_start;

	  /**
	   * Représente la présence ou non d'obstacle
	   */
	  Obstacle m_obstacle;

	  /**
	   * Used to publish on "Protokrot/start"
	   */
	  ros::Publisher     start_pub;

	  /**
	   * Used to publish on "Protokrot/start"
	   */
	  ros::Publisher     color_pub;

	  /**
	   * Used to publish on "Protokrot/start"
	   */
	  ros::Publisher     obstacle_pub;

	  /**
	   * used to call onUpdate
	   * Attention, pretty inaccurate
	   */
	  wxTimer* update_timer_;

	  /**
	   * called by timer. Manage ros::spinOnce
	   */
	  void onUpdate(wxTimerEvent& evt);

	  /**
	   * called by PaintEvent.
	   * Paint table then robot
	   */
	  void onPaint(wxPaintEvent& evt);

	  /**
	   * Called when the start button is trigerred
	   */
	  void onStart(wxCommandEvent& event);

	  /**
	   * Called when the color button is trigerred
	   */
	  void onColor(wxCommandEvent& event);

	  /**
	   * Called when the obstacle button is trigerred
	   */
	  void onObstacle(wxCommandEvent& event);

	private:
		DECLARE_EVENT_TABLE()

	};



}

#endif
