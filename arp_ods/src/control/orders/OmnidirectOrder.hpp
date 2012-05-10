/*
 * OmnidirectOrder.hpp
 *
 *  Created on: 210 april 2012
 *      Author: RMO
 */

#ifndef OMNIDIRECTORDER_HPP_
#define OMNIDIRECTORDER_HPP_

#include "MotionOrder.hpp"
#include "math/math.hpp"
#include <boost/shared_ptr.hpp>
#include <math/core>


namespace arp_ods{ namespace orders
{

class OmnidirectOrder: public MotionOrder
{
    public:

        /**
         * Nécessaire pour éviter ;
         * http://eigen.tuxfamily.org/dox/TopicStructHavingEigenMembers.html
         */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OmnidirectOrder();
        OmnidirectOrder(MotionOrder);

        /**
         * Override to define specific parameters
         */
        static boost::shared_ptr<MotionOrder> createOrder( const OrderGoalConstPtr &goal, arp_math::Pose2D currentPose, orders::config conf  );

        /**
         *
         */
        virtual arp_math::Twist2D computeSpeed(arp_math::Pose2D currentPosition, double dt);

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
        Twist2D computeRunTwist(arp_math::Pose2D currentPosition);

        /*
         * twist of precedent turn
         */
        arp_math::Twist2D m_v_correction_old;
        /*
         * twist at moment we entered the approach zone
         */
        arp_math::Twist2D m_twist_approach;
        /*
         * angle of speed for precedent turn
         */
        double m_angle_speedcorrection_old;



        //surcharges
        void switchRun(arp_math::Pose2D currentPosition);
        void switchApproach(arp_math::Pose2D currentPosition);

        static const double TIMELAG=0.060;
private:
    Pose2D getPositionError_RobotRef(arp_math::Pose2D currentPosition);
    Twist2D computeApproachTwist(arp_math::Pose2D currentPosition);

};

}}

#endif /* OMNIDIRECTORDER_HPP_ */
