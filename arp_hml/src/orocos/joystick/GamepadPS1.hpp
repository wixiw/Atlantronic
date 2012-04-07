/*
 * ManettePs1.hpp
 *
 *  Created on: 29/09/10
 *      Author: wla
 */

#ifndef GAMEPAD_PS1_HPP_
#define GAMEPAD_PS1_HPP_

#include "orocos/joystick/Joystick.hpp"

namespace arp_hml
{
    class GamepadPS1: public Joystick
    {
    public:
        GamepadPS1(const std::string& name);

        /** Name of the joystick drive file usually /dev/input/event0 */
        std::string propEventName;

        /** Vibreur */
        InputPort<bool> inRumble;

        /** Bouton 1 pouce droit */
        OutputPort<bool> outButton1;
        /** Bouton 2 pouce droit */
        OutputPort<bool> outButton2;
        /** Bouton 3 pouce droit */
        OutputPort<bool> outButton3;
        /** Bouton 4 pouce droit */
        OutputPort<bool> outButton4;

        /** Gachette haute gauche */
        OutputPort<bool> outTrigger5;
        /** Gachette haute droite */
        OutputPort<bool> outTrigger6;
        /** Gachette basse gauche */
        OutputPort<bool> outTrigger7;
        /** Gachette basse droite */
        OutputPort<bool> outTrigger8;

        /** Bouton "select" */
        OutputPort<bool> outButton9;
        /** Bouton "start"*/
        OutputPort<bool> outButton10;

        /** Axe X de la croix directionnelle : dirigé de gauche à droite sur la manette entre [-1;1] */
        OutputPort<double> outPadX;
        /** Axe y de la croix directionnelle : dirigé de bas en haut sur la manette entre [-1;1] */
        OutputPort<double> outPadY;
        /** Distance du joystick par rapport au neutre [0;sqrt(2)] */
        OutputPort<double> outPadXYDistance;
        /** Angle du joystick par rapport à x, compté positif vers y en radians */
        OutputPort<double> outPadXYAngle;

        OutputPort<double> outX1;
        OutputPort<double> outY1;
        OutputPort<double> outX2;
        OutputPort<double> outY2;

        /**
         * Surchargée pour calculer les distance et angles du joystick par rapport au neutre
         */
        void updateHook();

    protected:
        virtual bool checkIdentity();
        virtual void buttonEvent( struct js_event js );
        virtual void axisEvent( struct js_event js );
        virtual void initEvent( struct js_event js );

        /**
         * Override to no check
         */
        virtual bool checkInputsPorts();

        /**
         *
         */
        int m_event_fd;
    };
}

#endif /* GAMEPAD_PS1_HPP_ */
