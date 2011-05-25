/*
 * ModeSelector.hpp
 *
 *  Created on: 24 mai 2011
 *      Author: wla
 */

#ifndef MODESELECTOR_HPP_
#define MODESELECTOR_HPP_

#include <arp_core/Pose.h>

using namespace arp_core;

namespace arp_ods
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
         * Call this function every cycle to check if a new mode is available.
         * If a mode is available, the new mode is automatically entered
         */
        void switchMode(arp_core::Pose currentPosition);

        /**
         * Switch the mode back to MODE_INIT
         * be carefull when doing this.
         */
        void resetMode();

        /**
         * Returns the distance to the m_endPose
         * @param currentPosition : current position of the robot
         */
        virtual double getRemainingDistance(arp_core::Pose currentPosition);

        /**
         * Returns the angle difference (normalized) between m_endPose and currentPosition
         * @param currentPosition : current position of the robot
         */
        virtual double getRemainingAngle(arp_core::Pose currentPosition);

        /**
         * Returns the distance from the m_beginPose
         * @param currentPosition : current position of the robot
         */
        virtual double getCoveredDistance(arp_core::Pose currentPosition);

        /**
         * Returns the Begin pose m_beginPose
         */
        arp_core::Pose getBeginPose() const;

        /**
         * Returns the End pose m_endPose
         */
        arp_core::Pose getEndPose() const;

        /**
         * Returns the pass mode
         */
        bool getPass() const;

        /**
         * Returns the current mode m_currentMode
         */
        mode getMode() const;

        double getRadiusApproachZone() const;
        double getRadiusInitZone() const;
        double getAngleAccuracy() const;
        double getDistanceAccurancy() const;

        /**
         * Define the pass mode
         */
        void setPass(bool pass);

        void setRadiusApproachZone(double m_radiusApproachZone);
        void setRadiusInitZone(double m_radiusInitZone);

        void setAngleAccuracy(double m_angleAccuracy);
        void setDistanceAccurancy(double m_distanceAccurancy);
        void setBeginPose(arp_core::Pose beginPose);
        void setEndPose(arp_core::Pose endPose);
        void setPassTimeout(double timeout);
    protected:
        /** Pose of the expected begin of the move */
        arp_core::Pose m_beginPose;

        /** Pose of the expected end of the move */
        arp_core::Pose m_endPose;

        /** motion without stop at the end */
        bool m_pass;

        /** mode of operation*/
        mode m_currentMode;

        /** This parameter defines the INIT mode area in m*/
        double m_radiusInitZone;

        /** This parameter defines the APPROACH mode area in m*/
        double m_radiusApproachZone;

        /** This parameter defines the condtion in distance to end motion in m*/
        double m_distanceAccurancy;

        /** This parameter defines the condtion in angle to end motion in rad*/
        double m_angleAccuracy;

        /** Date at which we entered the PASS mode */
        double m_passTime;

        /** Pass timeout */
        double m_passTimeout;

        /** Compute the distance between two Poses a and b
         * TODO WLA : je n'arrive pas à le mettre dans math*/
        double distance(arp_core::Pose a, arp_core::Pose b);
        double angle(arp_core::Pose a, arp_core::Pose b);

        /**
         * This function is called by switchMode when m_currentMode==MODE_INIT
         */
        virtual void switchInit(arp_core::Pose currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_RUN
         */
        virtual void switchRun(arp_core::Pose currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_APPROACH
         */
        virtual void switchApproach(arp_core::Pose currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_DONE
         */
        virtual void switchDone(arp_core::Pose currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_ERROR
         */
        virtual void switchError(arp_core::Pose currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_PASS
         */
        virtual void switchPass(arp_core::Pose currentPosition);

};

}

#endif /* MODESELECTOR_HPP_ */
