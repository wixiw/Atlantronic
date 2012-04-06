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

namespace arp_ods
{

enum OrderType
{
    NO_ORDER, STAY_IN_POSITION, TRANSLATE, ROTATE, FANTOM
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

class MotionOrder: public arp_ods::ModeSelector
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

        /**
         * Compute the motor set points according to the current mode.
         * @param currentPosition : current Robot position
         * @param dt : time since last call
         */
        virtual arp_math::Twist2D computeSpeed(arp_math::Pose2D currentPosition, double dt);

        /**
         * Use this to filter begin, end and current point with m_reverse
         * If m_reverse is true, p.theta = p.theta+PI. p.x and p.y are unchanged.
         * In any case a normalization in [-PI;PI] is done so you should use this function in any case
         * @param p : pose to convert
         */
        arp_math::Pose2D reversePosition(arp_math::Pose2D p);

        /**
         * Factory to create an order with default parameters from the order
         * TODO il faudrait une vraie Factory en dehors, parce que là c'est un peu Joe La Bricole
         * @param goal : action lib goal to process
         * @param currentPose : current position of the robot
         * @param conf : automation parameters (gains)
         * @return : a MotionOrder to execute
         */
        static boost::shared_ptr<MotionOrder> createOrder( const OrderGoalConstPtr &goal, arp_math::Pose2D currentPose, order::config conf  );

        /**
         * Returns the type of the order
         */
        OrderType getType() const;

        /**
         * Returns a string according to the current type
         */
        std::string getTypeString() const;

        /**
         * Define the reverse mode
         */
        void setReverse(bool reverse);

        /**
         * Define the id of the order. If a unique ID is set,
         * this allows to distinguish unique orders
         */
        void setId(int id);

    protected:
        /** type of the current order */
        OrderType m_type;
        /** reverse type of motion */
        bool m_reverse;
        /** unique ID of the order */
        int m_id;


};

}

#endif /* MOTIONORDER_HPP_ */
