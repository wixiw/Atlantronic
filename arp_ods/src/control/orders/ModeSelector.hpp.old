/*
 * ModeSelector.hpp
 *
 *  Created on: 24 mai 2011
 *      Author: wla
 */

#ifndef MODESELECTOR_HPP_
#define MODESELECTOR_HPP_

#include "OrderConfig.hpp"
#include <math/core>
#include <models/core>

namespace arp_ods{ namespace orders
{

enum mode
{
    MODE_INIT = 1, MODE_RUN = 2, MODE_APPROACH = 3, MODE_DONE = 4, MODE_ERROR = 5, MODE_PASS = 6
};

/**
 * This is a mode selector to handle several modes in a move : INIT/RUN/APPROACH/PASS/DONE
 * It contains datas which are more MotionOrder related, but this is the simplest implementation
 * in the given time. This code is really open to refactor
 */
class ModeSelector
{
    public:

        ModeSelector();

        /**
         * Define configurable attributes
         */
        virtual void setConf(arp_ods::orders::config conf);

        /**
         * Call this function every cycle to check if a new mode is available.
         * If a mode is available, the new mode is automatically entered
         */
        void switchMode(arp_math::UbiquityMotionState currentMotionState);

        /**
         * Switch the mode back to MODE_INIT
         * be careful when doing this.
         */
        void resetMode();

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
         * Returns the current mode m_currentMode
         */
        mode getMode() const;



        /**
         * Define the pass mode
         */
        void setPass(bool pass);
        void setPassSpeed(double passSpeed);

        void setBeginMotionState(arp_math::UbiquityMotionState beginMotionState);
        void setEndMotionState(arp_math::UbiquityMotionState endMotionState);
        void setCpoint(arp_math::Pose2D cpoint);


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
        mode m_currentMode;

        /** configuration */
        config m_conf;

        /** Date at which we entered the PASS mode */
        long double m_passTime;

        /** Date at which we entered the INIT mode **/
        long double m_initTime;

        /** Date at which we entered the approach mode **/
        long double m_approachTime;

        /** Date at which we entered the run mode **/
        long double m_runTime;

        /**
         * This function is called by switchMode when m_currentMode==MODE_INIT
         */
        virtual void switchInit(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_RUN
         */
        virtual void switchRun(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_APPROACH
         */
        virtual void switchApproach(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_DONE
         */
        virtual void switchDone(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_ERROR
         */
        virtual void switchError(arp_math::UbiquityMotionState currentMotionState);

        /**
         * This function is called by switchMode when m_currentMode==MODE_PASS
         */
        virtual void switchPass(arp_math::UbiquityMotionState currentMotionState);

};

}}

#endif /* MODESELECTOR_HPP_ */
