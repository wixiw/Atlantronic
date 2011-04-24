/*
 * ManettePs1.hpp
 *
 *  Created on: 29/09/10
 *      Author: wla
 */

#ifndef LOGITECH_EXTREME_3D_HPP_
#define LOGITECH_EXTREME_3D_HPP_

#include "Joystick.hpp"

namespace arp_hml
{
    using namespace RTT;

    class LogitechExtreme3D: public Joystick
    {
    public:
        LogitechExtreme3D(const std::string& name);

        /** Bouton 1 Gachette */
        OutputPort<bool> outButton1;
        /** Bouton 2 Tir Alternatif */
        OutputPort<bool> outButton2;
        /** Bouton 3 pouce droit */
        OutputPort<bool> outButton3;
        /** Bouton 4 pouce droit */
        OutputPort<bool> outButton4;
        /** Bouton 5 pouce droit  */
        OutputPort<bool> outButton5;
        /** Bouton 6 pouce droit  */
        OutputPort<bool> outButton6;

        /** Bouton 7 socle */
        OutputPort<bool> outButton7;
        /** Bouton 8 socle */
        OutputPort<bool> outButton8;
        /** Bouton 9 socle */
        OutputPort<bool> outButton9;
        /** Bouton 10 socle */
        OutputPort<bool> outButton10;
        /** Bouton 11 socle */
        OutputPort<bool> outButton11;
        /** Bouton 12 socle */
        OutputPort<bool> outButton12;

        /** Axe X du manche : dirigé de gauche à droite sur la manette entre [-1;1] */
        OutputPort<double> outX;
        /** Axe y du manche : dirigé de bas en haut sur la manette entre [-1;1] */
        OutputPort<double> outY;
        /** Axe z du manche : rotation horaire entre [-1;1] */
        OutputPort<double> outZ;
        /** Axe des gaz du manche : rotation entre [-1;1] signe "+" = -1, signe "-" = +1*/
        OutputPort<double> outThrottle;
        /** Croix directionnelle du haut du joystick : axe gauche-droite */
        OutputPort<double> outTopX;
        /** Croix directionnelle du haut du joystick : axe haut-bas */
        OutputPort<double> outTopY;
        /** Distance du joystick par rapport au neutre [0;sqrt(2)] */
        OutputPort<double> outXYDistance;
        /** Angle du joystick par rapport à x, compté positif vers y en radians */
        OutputPort<double> outXYAngle;
        /** Distance de la croix directionnelle par rapport au neutre [0;sqrt(2)] */
        OutputPort<double> outTopXYDistance;
        /** Angle de la croix directionnelle par rapport à x, compté positif vers y en radians */
        OutputPort<double> outTopXYAngle;

        /**
         * Surchargée pour calculer les distance et angles du joystick par rapport au neutre
         */
        void updateHook();

    protected:
        virtual bool checkIdentity();
        virtual void buttonEvent( struct js_event js );
        virtual void axisEvent( struct js_event js );
        virtual void initEvent( struct js_event js );
    };
}

#endif /* LOGITECH_EXTREME_3D_HPP_ */
