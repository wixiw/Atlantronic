/*
 * ICRSpeedBuffer.hpp
 *
 *  Created on: Apr 30, 2012
 *      Author: Romain Moulin
 */

#ifndef ICRSPEEDBUFFER_HPP_
#define ICRSPEEDBUFFER_HPP_

#include <math/core>
#include <models/core>

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace std;

namespace arp_ods
{

class ICRSpeedBuffer
{
    public:
        ICRSpeedBuffer();
        virtual ~ICRSpeedBuffer();

        void addICRSpeed(ICRSpeed speed, double dt);
        void removeICRSpeed(ICRSpeed & twist, double & dt);

    protected:
        static const int BUFFERSIZE=512;
        ICRSpeed m_ICRSpeedList[BUFFERSIZE];
        double m_dtList[BUFFERSIZE];
        int m_curIndex;


};

}
#endif /* ICRSPEEDBUFFER_HPP_ */
