/*
 * TwistBuffer.hpp
 *
 *  Created on: Apr 30, 2012
 *      Author: Romain Moulin
 */

#ifndef TWISTBUFFER_HPP_
#define TWISTBUFFER_HPP_

#include <math/core>
#include <models/core>
#include "time/ArdTime.hpp"

namespace arp_ods
{

class TwistBuffer
{
    public:
        TwistBuffer();
        virtual ~TwistBuffer();

        void addTwist(arp_math::Twist2D twist, arp_time::ArdTimeDelta dt);
        void removeTwist(arp_math::Twist2D & twist, arp_time::ArdTimeDelta & dt);

    protected:
        static const int BUFFERSIZE=512;
        arp_math::Twist2D m_twistList[BUFFERSIZE];
        arp_time::ArdTimeDelta m_dtList[BUFFERSIZE];
        int m_curIndex;


};

}
#endif /* TWISTBUFFER_HPP_ */
