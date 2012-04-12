/*
 * ModeSelector.hpp
 *
 *  Created on: 24 mai 2011
 *      Author: wla
 */

#ifndef MODESELECTOR_HPP_
#define MODESELECTOR_HPP_

#include <math/core>

namespace arp_ods{ namespace orders
{

enum mode
{
    MODE_INIT = 1, MODE_RUN = 2, MODE_APPROACH = 3, MODE_DONE = 4, MODE_ERROR = 5, MODE_PASS = 6
};

    struct config
    {
        /** This default parameter defines the APPROACH mode area in m*/
        double RADIUS_APPROACH_ZONE;

        /** This default parameter defines the condition in distance to end motion in m*/
        double DISTANCE_ACCURACY;

        /** This default parameter defines the condition in angle to end motion in rad*/
        double ANGLE_ACCURACY;

        /** Pass default timeout */
        double PASS_TIMEOUT;

        /** Order timeout*/
        double ORDER_TIMEOUT;

        /**Constantes issues des rosparam*/
         double FANTOM_COEF;
         double VEL_FINAL;
         double ROTATION_GAIN;
         double ROTATION_D_GAIN;
         double TRANSLATION_GAIN;

         double LIN_VEL_MAX;
         double ANG_VEL_MAX;
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
         * Define configurable attributes to their default values
         */
        virtual void setDefaults(arp_ods::orders::config conf);

        /**
         * Call this function every cycle to check if a new mode is available.
         * If a mode is available, the new mode is automatically entered
         */
        void switchMode(arp_math::Pose2D currentPosition);

        /**
         * Switch the mode back to MODE_INIT
         * be carefull when doing this.
         */
        void resetMode();

        /**
         *  will test for timeout and if true, set mode to ERROR
         */
        void testTimeout();

        /**
         * Returns the distance to the m_endPose
         * @param currentPosition : current position of the robot
         */
        virtual double getRemainingDistance(arp_math::Pose2D currentPosition);

        /**
         * Returns the angle difference (normalized) between m_endPose and currentPosition
         * @param currentPosition : current position of the robot
         */
        virtual double getRemainingAngle(arp_math::Pose2D currentPosition);

        /**
         * Returns the distance from the m_beginPose
         * @param currentPosition : current position of the robot
         */
        virtual double getCoveredDistance(arp_math::Pose2D currentPosition);

        /**
         * Returns the Begin pose m_beginPose
         */
        arp_math::Pose2D getBeginPose() const;

        /**
         * Returns the End pose m_endPose
         */
        arp_math::Pose2D getEndPose() const;

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
        void setBeginPose(arp_math::Pose2D beginPose);
        void setEndPose(arp_math::Pose2D endPose);
        void setCpoint(arp_math::Pose2D cpoint);
        void setPassTimeout(double timeout);
        void setOrderTimeout(double timeout);

    protected:
        /** Pose of the expected begin of the move */
        arp_math::Pose2D m_beginPose;

        /** Pose of the expected end of the move */
        arp_math::Pose2D m_endPose;

        /** Control point on the robot */
        arp_math::Pose2D m_cpoint;

        /** motion without stop at the end */
        bool m_pass;

        /** mode of operation*/
        mode m_currentMode;

        /** This parameter defines the INIT mode area in m*/
        double m_radiusInitZone;

        /** This parameter defines the APPROACH mode area in m*/
        double m_radiusApproachZone;

        /** This parameter defines the condition in distance to end motion in m*/
        double m_distanceAccuracy;

        /** This parameter defines the condition in angle to end motion in rad*/
        double m_angleAccuracy;

        /** Date at which we entered the PASS mode */
        double m_passTime;

        /** Date at which we entered the INIT mode **/
        double m_initTime;

        /** Pass timeout */
        double m_passTimeout;

        /** order timeout */
        double m_orderTimeout;

        /**
         * This function is called by switchMode when m_currentMode==MODE_INIT
         */
        virtual void switchInit(arp_math::Pose2D currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_RUN
         */
        virtual void switchRun(arp_math::Pose2D currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_APPROACH
         */
        virtual void switchApproach(arp_math::Pose2D currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_DONE
         */
        virtual void switchDone(arp_math::Pose2D currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_ERROR
         */
        virtual void switchError(arp_math::Pose2D currentPosition);

        /**
         * This function is called by switchMode when m_currentMode==MODE_PASS
         */
        virtual void switchPass(arp_math::Pose2D currentPosition);

};

}}

#endif /* MODESELECTOR_HPP_ */
