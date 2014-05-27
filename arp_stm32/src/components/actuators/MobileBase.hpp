/*
 * MobileBase.hpp
 *
 *  Created on: May 26, 2014
 *      Author: ard
 */

#ifndef MOBILEBASE_HPP_
#define MOBILEBASE_HPP_

#include "components/taskcontexts/Stm32TaskContext.hpp"
#include "linux/tools/robot_interface.h"

namespace arp_stm32
{

class MobileBase: public Stm32TaskContext
{
    public:
        MobileBase();
        virtual ~MobileBase();

        /****************************************************************
         * Interface Orocos
         ****************************************************************/

        bool configureHook();
        void updateHook();

        /** Ce port est publié à la fin de la syncronisation, des composants qui sont connectés aux ports de données peuvent
         * se trigger en eventPort sur outClock. Il contient la date des mesures sur le CAN (date de l'envoit du message SYNC)
         */
        OutputPort<arp_time::ArdAbsoluteTime> outClock;

        /**
         * Ce port contient un condensé de toutes les mesures Hml a une date identique pour tous les moteurs.
         */
        OutputPort<arp_model::MotorState> outMotorMeasures;


        /****************************************************************
         * Interface ROS
         ****************************************************************/

    protected:
        void createOrocosInterface();
        RobotInterface& m_robotItf;
};

}

#endif /* MOBILEBASE_HPP_ */
