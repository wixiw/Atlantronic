/*
 * ScriptTeleop.hpp
 *
 *  Created on: Apr 5, 2012
 *      Author: ard
 */

#ifndef SCRIPTTELEOP_HPP_
#define SCRIPTTELEOP_HPP_

#include "taskcontexts/OdsTaskContext.hpp"
#include <math/core>


namespace arp_ods
{

class ScriptTeleop: public OdsTaskContext
{
    public:
        ScriptTeleop(const std::string& name);
        void updateHook();

    protected:
        double propLinSpeed;
        double propAngSpeed;
        int attrMode;
        arp_math::Twist2D attrTwistCmd;
        RTT::OutputPort<arp_math::Twist2D> outTwistCmd;
        double m_direction;
};

} /* namespace arp_ods */
#endif /* SCRIPTTELEOP_HPP_ */
