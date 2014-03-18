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

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace std;

namespace arp_ods
{

class TwistBuffer
{
    public:
        TwistBuffer();
        virtual ~TwistBuffer();

        void addTwist(Twist2D twist, double dt);
        void removeTwist(Twist2D & twist, double & dt);

    protected:
        static const int BUFFERSIZE=512;
        Twist2D m_twistList[BUFFERSIZE];
        double m_dtList[BUFFERSIZE];
        int m_curIndex;


};

}
#endif /* TWISTBUFFER_HPP_ */
