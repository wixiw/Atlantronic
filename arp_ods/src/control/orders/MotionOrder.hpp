/*
 * MotionOrder.hpp
 *
 *  Created on: 23 mai 2011
 *      Author: wla
 */

#ifndef MOTIONORDER_HPP_
#define MOTIONORDER_HPP_

#include "ModeSelector.hpp"
#include <arp_ods/OrderAction.h>
#include <boost/shared_ptr.hpp>
#include <math/core>
#include "control/ICRSpeedBuffer.hpp"
#include "control/OnlineTrajectoryGenerator.hpp"


using namespace arp_math;

namespace arp_ods{ namespace orders
{

enum OrderType
{
    NO_ORDER, STAY_IN_POSITION, TRANSLATE, ROTATE, FANTOM, OMNIDIRECT2, OPENLOOP, REPLAY
};


/**
 * A motion order is containing all required information to execute an order.
 * You have to call the computeSpeed function periodically to execute the order.
 *
 * The order is a child of ModeSelector which is owning the default mode behavior. See its documentation
 * as some required stuff for a special order is in it (pass,begin/end point, accuracy...)
 *
 * TODO WLA : implémenter l'identifiant unique
 */

class MotionOrder: public ModeSelector
{
    public:
        /**
         * Copy constructor
         */
        MotionOrder(const MotionOrder& order);

        /**
         * Default constructor
         */
        MotionOrder();

        ~MotionOrder();

        /**
         * Compute the motor set points according to the current mode.
         * @param currentMotionState : current Robot position
         * @param dt : time since last call
         */
        virtual ICRSpeed computeSpeed(UbiquityMotionState currentMotionState,UbiquityParams params, double dt);

        /**
         * Factory to create an order with default parameters from the order
         * TODO il faudrait une vraie Factory en dehors, parce que là c'est un peu Joe La Bricole
         * @param goal : action lib goal to process
         * @param currentMotionState : current motion state of the robot
         * @param conf : automation parameters (gains)
         * @return : a MotionOrder to execute
         */
        static boost::shared_ptr<MotionOrder> createOrder( const OrderGoalConstPtr &goal, UbiquityMotionState currentMotionState, orders::config conf  );

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
        void setICRSpeedBuffer(ICRSpeedBuffer twistBuffer );
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
        double  outDEBUG8;
        double  outDEBUG9;
        double  outDEBUG10;

        double attrGain;

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
        /** type of the current order */
        OrderType m_type;
        /** twist in case of openloop */
        Twist2D m_openloop_twist;
        /** duration of the command in case of openloop */
        double m_openloop_duration;
        /** unique ID of the order */
        int m_id;
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

};

}}

#endif /* MOTIONORDER_HPP_ */
