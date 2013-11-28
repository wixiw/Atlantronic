/*
 * OmnidirectOrder.hpp
 *
 *  Created on: 14 sept 2013
 *      Author: RMO
 */

#ifndef OMNIDIRECTORDER2_HPP_
#define OMNIDIRECTORDER2_HPP_

#include "MotionOrder.hpp"
#include "control/OnlineTrajectoryGenerator.hpp"

#include "math/math.hpp"
#include <boost/shared_ptr.hpp>
#include <math/core>


namespace arp_ods{ namespace orders
{

class OmnidirectOrder2: public MotionOrder
{
    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OmnidirectOrder2();
        OmnidirectOrder2(MotionOrder);

        /**
         * Override to define specific parameters
         */
        static boost::shared_ptr<MotionOrder> createOrder( const OrderGoalConstPtr &goal, arp_math::Pose2D currentPose, orders::config conf  );

        /**
         *
         */
        virtual arp_math::Twist2D computeSpeed(Pose2D currentPosition,MotorState motorState,UbiquityParams params, double dt);

        /*
         * returns the error on position between actual an objective,  in table referential
         */
        arp_math::Pose2D getPositionError(arp_math::Pose2D currentPosition);



    protected:
        /*
         * saturate the twist for acceleration and max speed
         */
        Twist2D saturateTwist(Twist2D v_correction_ref, double dt);
        /*
         * compute the usual "mode run" twist
         */
        ICRSpeed computeRunTwist(arp_math::Pose2DNorm currentPosition,ICRSpeed curICRSpeed,double dt);
        double profileRo(double distance,ICRSpeed curICRSpeed);
        double profileRoJerking(double distance, ICRSpeed curICRSpeed);

        /*
         * twist of precedent turn
         */
        arp_math::Twist2D m_v_correction_old;
        /*
         * twist and error at moment we entered the approach zone
         */
        arp_math::Twist2D m_twist_approach;
        arp_math::Pose2D m_error_approach;
        arp_math::Pose2D m_pose_approach;
        /*
         * twist when we entered the reconfigurator
         */
        arp_math::Twist2D m_twist_init;
        bool m_twist_init_registered;

        double m_normalizedError;
        double m_normalizedError_old;

        /*
         * the maximum speed for this order
         */
        double m_vmax_order;



        //speed at last turn - used for acceleration computation
        ICRSpeed m_oldICRSpeed;
        double m_predictedAcc;
        double m_lastRo;

        //surcharges
        void switchRun(arp_math::Pose2D currentPosition);

        static const double TIMELAG=0.030;
        static const double DIST_SMOOTH=0.100;
        static const double SUP_TIME_FOR_ROTATION=1.0;
        static const double ANGLE_CHGT_ASSERV=0.17;
        static const double RECONF_TIME=0.0;
        static const double MIN_LIN_DEC=0.2;
        static const double TIMEOUTMAX=10;
        static const double TIMEOUTMIN=5;



private:
    Pose2D getPositionError_RobotRef(arp_math::Pose2D currentPosition);
    void decideSmoothNeeded(arp_math::Pose2D & currentPosition);
    Pose2DNorm getPositionInNormalRef(Pose2D currentPosition);
    double getParkinsonLimitationFactor(double distance);
};

}}

#endif /* OMNIDIRECTORDER_HPP_ */
