/*
 * FantomOrder.hpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef FANTOMORDER_HPP_
#define FANTOMORDER_HPP_

#include "MotionOrder.hpp"
#include "math/math.hpp"
#include <boost/shared_ptr.hpp>
#include <math/core>

using namespace boost;

namespace arp_ods
{

class FantomOrder: public arp_ods::MotionOrder
{
    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        FantomOrder();
        FantomOrder(MotionOrder);

        /**
         * Define configurable attributes to their default values
         */
        virtual void setDefaults(order::config conf);

        /**
         * Override to define specific parameters
         */
        static shared_ptr<MotionOrder> createOrder( const OrderGoalConstPtr &goal, arp_math::Pose2D currentPose, order::config conf  );

        /**
         *
         */
        virtual arp_math::Twist2D computeSpeed(arp_math::Pose2D currentPosition, double dt);

        /**
         * this is the restriction of linear speed due to angle error. the coefficient is between -1 and 1
         */
        static double linearReductionCoef(double error);

        /**
         * Derivated to handle the different computation in approach mode
         * @param currentPosition : current position of the robot
         * @return if m_mode == Approach : distance of the robot to the line perpendicular to the m_endPoint heading
         * in any other case it simply returns the euclidian distance to the end point
         */
        double getRemainingDistance(arp_math::Pose2D currentPosition);

        /**
         * Derivated to handle the different computation in approach mode
         * @param currentPosition : current position of the robot
         * @param remaining_distance : the distance to the point from getRemainingDistance
         * @return if m_mode == Approach : ...//TODO WLA documenter
         */
        double getRemainingAngle(arp_math::Pose2D currentPosition, double remaining_distance);

        /**
         * Calcule l'erreur derivée en cap
         * TODO : si ici que sont fait les timing, faut-il extraire pour un éventuel dérivé de position ?
         * @param currentPosition : current position of the robot
         * @param angle_error : the new angle error
         * @param dt : time since last call
         */
        double getDerivatedAngleError(arp_math::Pose2D currentPosition, double angle_error, double dt);

        /**
         * When errors are defined, call this function to have the linear speed command
         * @param distance_error : the longitudinal distance in mm
         * @param angle_error : the angle arror in rad
         * @return the linear speed in m/s
         */
        double getLinearSpeedCmd(double distance_error, double angle_error);

        /**
         * @param angle_error : the angle arror in rad
         * @param angle_d_error : derivated angle error in rad/s
         * @return the angular speed in rad/s
         */
        double getAngularSpeedCmd(double angle_error, double d_angle_error);

        /** Getters and setters */
        double getFANTOM_COEF() const;
        double getROTATION_D_GAIN() const;
        double getROTATION_GAIN() const;
        double getTRANSLATION_GAIN() const;
        double getVEL_FINAL() const;
        void setFANTOM_COEF(double FANTOM_COEF);
        void setROTATION_D_GAIN(double ROTATION_D_GAIN);
        void setROTATION_GAIN(double ROTATION_GAIN);
        void setTRANSLATION_GAIN(double TRANSLATION_GAIN);
        void setVEL_FINAL(double VEL_FINAL);

    protected:
        /**
         * Redifined because there is no init for this order
         */
        virtual void switchInit(arp_math::Pose2D currentPosition);

        /**
         * Redefined to keep trac of the speed when entering pass mode
         * TODO shouldn't this go into the MotionControl class ?
         */
        virtual void switchRun(arp_math::Pose2D currentPosition);

        /** keep track of the former loop value of angle_error, to compute derivative */
        double old_angle_error;
        /** buffer to keep track of the speed when entering pass mode */
        arp_math::Twist2D m_passSpeed;
        /** buffer to keep memory of last sended command */
        arp_math::Twist2D m_lastSpeedCmd;

       /**Constantes issues des rosparam*/
        double FANTOM_COEF;
        double VEL_FINAL;
        double ROTATION_GAIN;
        double ROTATION_D_GAIN;
        double TRANSLATION_GAIN;
};

}

#endif /* FANTOMORDER_HPP_ */
