/*
 * ActuatorGraphicsNode.hpp
 *
 *  Created on: 11 mars 2014
 *      Author: wla
 */

#ifndef ARP_HML_ACTUATORSFRAME_2014_HPP
#define ARP_HML_ACTUATORSFRAME_2014_HPP

#define wxUSE_GUI 1
#include <wx/wx.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "RosLinkedButton.hpp"

using namespace std_msgs;

namespace arp_hml
{
    /** \ingroup arp_hml
     * \nonstableyet
     *
     * \class ActuatorsFrame_2014
     *
     * \brief Actuator Simulation IHM for y2014
     *
     * Allow user to interact as if it was playing with robot 2014 specific hardware
     *
     */

    class ActuatorsFrame_2014: public wxFrame
    {
    public:

        /**
         * Default constructor
         * A timer is set width default_dt_ms time step
         * onUpdate is then called every default_dt_ms
         */
        ActuatorsFrame_2014();
        ~ActuatorsFrame_2014(){};

        static const int ID_LEFT_SUCKER_FIRE_TOR_HANDLER = 1;
        static const int ID_RIGHT_SUCKER_FIRE_TOR_HANDLER = 2;
        static const int ID_LEFT_SUCKER_SCAN_TOR_HANDLER = 3;
        static const int ID_RIGHT_SUCKER_SCAN_TOR_HANDLER = 4;
        static const int ID_LEFT_ARM_BLOCKED_HANDLER = 5;
        static const int ID_RIGHT_ARM_BLOCKED_HANDLER = 6;

        void onLeftSuckerFireEvt(wxCommandEvent& event);
        void onRightSuckerFireEvt(wxCommandEvent& event);
        void onLeftArmScanEvt(wxCommandEvent& event);
        void onRightArmScanEvt(wxCommandEvent& event);
        void onLeftArmBlockedEvt(wxCommandEvent& event);
        void onRightArmBlockedEvt(wxCommandEvent& event);

    protected:

        /**
         * Simulator Nodehandle
         * Instanciated by GraphicsSimuFrame constructor
         */
        ros::NodeHandle m_nodeHandle;

        /**
         * Size of the main view
         */
        static const double length_in_pixel = 350.0;

        /**
         * Size of the main view
         */
        static const double width_in_pixel = 250.0;



        /**
         * Bouton représentant le capteur Omron visant sous la ventouse gauche à la verticale
         * Used to publish on "Ubiquity2014/leftArmSuckerTor"
         */
        RosLinkedButton m_leftSuckerTorButton;

        /**
         * Bouton représentant le capteur Omron visant sous la ventouse droite à la verticale
         */
        RosLinkedButton m_rightSuckerTorButton;

        /**
         * Button représentant le capteur Omron visant à gauche de la ventouse gauche à l'horizontal
         */
        RosLinkedButton m_leftScanTorButton;


        /**
         * Button représentant le capteur Omron visant à droite de la ventouse droite à l'horizontal
         */
        RosLinkedButton m_rightScanTorButton;

        /**
         * Button représentant un blocage du bras gauche
         */
        RosLinkedButton m_leftArmBlockedButton;

        /**
         * Button représentant un blocage du bras droit
         */
        RosLinkedButton m_rightArmBlockedButton;


        /**
         * Représente la position du bras gauche
         */
        //TODO XXX m_leftArmPosition;

        /**
         * Représente la position du bras droit
         */
        //TODO XXX m_rightArmPosition;



        /**
         * used to call onUpdate
         * Attention, pretty inaccurate
         */
        wxTimer m_updateTimer;

        /**
         * called by timer. Manage ros::spinOnce
         */
        void onUpdate(wxTimerEvent& evt);

        /**
         * called by PaintEvent.
         * Paint the view
         */
        void onPaint(wxPaintEvent& evt);


    private:
        DECLARE_EVENT_TABLE()

    };

}

#endif
