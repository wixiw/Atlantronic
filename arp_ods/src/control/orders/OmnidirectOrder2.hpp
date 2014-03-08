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
#include <models/core>

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

        OmnidirectOrder2(const OrderGoalConstPtr &goal, arp_math::UbiquityMotionState currentMotionState,
                UbiquityParams params);
        ~OmnidirectOrder2(){};


        /**
         *
         */
        //virtual arp_math::Twist2D computeSpeed(Pose2D currentPosition,MotorState motorState,UbiquityParams params, double dt);
        virtual ICRSpeed computeSpeed(arp_math::UbiquityMotionState currentMotionState, double dt);


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

        double profileRoJerking(double distance, ICRSpeed curICRSpeed, double roPass, double dt);

        /*  this little function creates the "cheat" on distance given to profile, to compensate for the delay in the loop
         *
         *                return
         *                  ^
         *                  |     /
         *                  |    /
         *        -----__________--------> realDistance
         *            /     |<--> distdelay
         *           /      |
         *
         *
         */
        double distanceModifier(double realDistance, double distanceDelay);

        /*
         * the maximum speed for this order
         */
        double m_vmax_order;



        //speed at last turn - used for acceleration computation
        ICRSpeed m_oldICRSpeed;

        //surcharges
        void switchInit(arp_math::UbiquityMotionState currentMotionState);
        void switchRun(arp_math::UbiquityMotionState currentMotionState);

        /*
         * distance at which we shall ask for smooth localisation so that we don't get crazy motion control
         */
        static const double DIST_SMOOTH=0.100;

        /*
         * distance at which we consider that the order is DONE
         */
        static const double RO_ACCURACY=0.005;

        /*
         * distance at which we will switch from a v=k. sqrt(dist) to v = k. dist motion control. this allow removing the unstability due to the infinite derivative of sqrt around 0
         */
        static const double DISTANCE_LINEAR_ASSERV=0.030;
        /*
         * deceleration to approach points
         */
        static const double DECELERATION=2.0;


private:
    Pose2D getPositionError_RobotRef(arp_math::Pose2D currentPosition);
    void decideSmoothNeeded(arp_math::Pose2D & currentPosition);
    Pose2DNorm getPositionInNormalRef(Pose2D currentPosition);
    double getParkinsonLimitationFactor(double distance);
    double getSpeedLimitationFactor(double distanceICRMove);
};

}}

#endif /* OMNIDIRECTORDER_HPP_ */
