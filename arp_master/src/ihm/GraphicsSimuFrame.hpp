/*
 * GraphicsSimuFrame.hpp
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

#include <arp_core/Pose.h>
#include <arp_core/MotionTarget.h>

#include "math/math.hpp"
#include <math/core>

#include <std_srvs/Empty.h>
#include <arp_core/Spawn.h>
#include <arp_core/Kill.h>
#include <map>

#include "GraphicsSimuRobot.hpp"

namespace arp_master
{
    /** \ingroup arp_master
     * \nonstableyet
     *
     * \class GraphicsSimuFrame
     *
     * \brief Table Simulation
     *
     * Manage physical and graphical simulation of table and robot
     * Should be divided in two parts
     *
     */

    class GraphicsSimuFrame: public wxFrame
    {

    private:

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
        static const double table_width_in_pixel = 400.0;
    public:

        /**
         * Default constructor
         * A timer is set width default_dt_ms time step
         * onUpdate is then called every default_dt_ms
         */
        GraphicsSimuFrame(std::string topicName);
        ~GraphicsSimuFrame();

    private:

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
         * clear the table of all trace
         */
        void clear();

        /**
         * used by clear service
         */
        bool clearCallback(std_srvs::Empty::Request&,
                std_srvs::Empty::Response&);

        /**
         * Simulator Nodehandle
         * Instanciated by GraphicsSimuFrame constructor
         */
        ros::NodeHandle nh_;

        /**
         * ServiceServer used to clear table of all trace
         */
        ros::ServiceServer clear_srv_;

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
        GraphicsSimuRobotPtr mRobot;

        /**
         * robot associated image
         */
        wxImage robot_image_;

        /*
         * motion target
         */
        ros::Subscriber m_motionTargetSub;
        arp_core::MotionTarget m_motionTarget;
        void motionTargetCallback(const arp_core::MotionTarget& m);

        void drawArrow(wxPaintDC & dc,double x, double y, double theta, double lenght);
        Vector2 real2Frame(Vector2 vect);
        void drawLineInDC(wxPaintDC & dc, Vector2 start,Vector2 end);
    };

}

#endif
