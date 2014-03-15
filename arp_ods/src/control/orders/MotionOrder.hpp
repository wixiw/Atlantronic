/*
 * MotionOrder.hpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef MOTIONORDER_HPP_
#define MOTIONORDER_HPP_

#include <models/core>
#include <math/core>
#include <boost/shared_ptr.hpp>
#include "control/ICRSpeedBuffer.hpp"
#include "control/OnlineTrajectoryGenerator.hpp"
#include <arp_ods/OrderAction.h>

using namespace arp_math;

enum Mode
{
    MODE_INIT = 1, MODE_RUN = 2, MODE_DONE = 4, MODE_ERROR = 5
};

enum OrderType
{
    STAY, OMNIDIRECT2, OPENLOOP, REPLAY
};

namespace arp_ods
{
namespace orders
{




/**
 * A motion order is containing all required information to execute an order.
 * You have to call the computeSpeed function periodically to execute the order.
 *
 *
 * TODO WLA : implémenter l'identifiant unique
 */

class MotionOrder
{
    public:
        MotionOrder(const OrderGoalConstPtr &goal, arp_math::UbiquityMotionState currentMotionState,
                UbiquityParams params);

        virtual ~MotionOrder();

        /**
         * Call this function every cycle to check if a new Mode is available.
         * If a Mode is available, the new Mode is automatically entered
         */
        void switchMode(arp_math::UbiquityMotionState currentMotionState);


        /**
         *  will test for timeout and if true, set mode to ERROR
         */
        void testTimeout();

        /**
         * Returns the distance to the m_endMotionState
         * @param currentPosition : current position of the robot
         */
        virtual double getRemainingDistance(arp_math::UbiquityMotionState currentMotionState);

        /**
         * Returns the angle difference (normalized) between m_endMotionState and currentPosition
         * @param currentPosition : current position of the robot
         */
        virtual double getRemainingAngle(arp_math::UbiquityMotionState currentMotionState);

        /**
         * Returns the distance from the m_beginMotionState
         * @param currentPosition : current position of the robot
         */
        virtual double getCoveredDistance(arp_math::UbiquityMotionState currentMotionState);

        /**
         * Returns the Begin pose m_beginMotionState
         */
        arp_math::UbiquityMotionState getBeginMotionState() const;

        /**
         * Returns the End pose m_endMotionState
         */
        arp_math::UbiquityMotionState getEndMotionState() const;

        /*
         * returns the control point
         */
        arp_math::Pose2D getCpoint() const;
        /**
         * Returns the pass mode
         */
        bool getPass() const;

        /**
         * Returns the current Mode m_currentMode
         */
        Mode getMode() const;

        /**
         * Define the pass Mode
         */
        void setPass(bool pass);
        void setPassSpeed(double passSpeed);

        void setBeginMotionState(arp_math::UbiquityMotionState beginMotionState);
        void setEndMotionState(arp_math::UbiquityMotionState endMotionState);
        void setCpoint(arp_math::Pose2D cpoint);

        /**
         * Compute the motor set points according to the current mode.
         * @param currentMotionState : current Robot position
         * @param dt : time since last call
         * Pure virtual function
         */
        virtual ICRSpeed computeSpeed(UbiquityMotionState currentMotionState, double dt)=0;



        /**
         * Returns the type of the order
         */
        OrderType getType() const;

        /**
         * Returns a string according to the current type
         */
        std::string getTypeString() const;

        /**
         * Define the id of the order. If a unique ID is set,
         * this allows to distinguish unique orders
         */
        void setId(int id);
        /*
         * set
         */
        void setICRSpeedBuffer(ICRSpeedBuffer twistBuffer);
        void setOTG(OnlineTrajectoryGenerator * OTG_);

        /*
         * DEBUG
         */
        double outDEBUG1;
        double outDEBUG2;
        double outDEBUG3;
        double outDEBUG4;
        double outDEBUG5;
        double outDEBUG6;
        double outDEBUG7;
        double outDEBUG8;
        double outDEBUG9;
        double outDEBUG10;

        /**
         * is smooth localization needed ?
         */
        bool m_smoothLocNeeded;
        /*
         * max speed (input)
         */
        double m_vmax_asked;
        void setVmax(double vmax);

    protected:
        /** MotionState of the expected begin of the move */
        arp_math::UbiquityMotionState m_beginMotionState;

        /** MotionState of the expected end of the move */
        arp_math::UbiquityMotionState m_endMotionState;

        /** Control point on the robot */
        arp_math::Pose2D m_cpoint;

        /** motion without stop at the end */
        bool m_pass;
        double m_passSpeed;

        /** mode of operation*/
        Mode m_currentMode;

        /** Date at which we entered the INIT Mode **/
        long double m_initTime;

        /** Date at which we entered the run mode **/
        long double m_runTime;

        /** date at which the order shall stop */
        double m_timeout;

        /**
         * This function is called by switchMode when m_currentMode==MODE_INIT
         */
        virtual void switchInit(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_RUN
         */
        virtual void switchRun(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_DONE
         */
        virtual void switchDone(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_ERROR
         */
        virtual void switchError(arp_math::UbiquityMotionState currentMotionState);





        /** type of the current order */
        OrderType m_type;
        /** twist in case of openloop */
        Twist2D m_openloop_twist;
        /** duration of the command in case of openloop */
        double m_openloop_duration;
        /*
         * buffer of twist for replaying backward
         */
        ICRSpeedBuffer m_twistBuffer;

        /**
         * error at precedent turn
         */
        Pose2D m_error_old;

        /*
         *  a pointer to the profile computation library
         */
        OnlineTrajectoryGenerator * OTG;

        /*
         * ubiquity parameters, given at creation of the order
         */
        UbiquityParams m_params;

};

}
}

#endif /* MOTIONORDER_HPP_ */
