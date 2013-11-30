/*
 * ICRSpeedBuffer.cpp
 *
 *  Created on: Apr 30, 2012
 *      Author: Romain Moulin
 */

#include "ICRSpeedBuffer.hpp"
#include <iostream>

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;
using namespace std;

ICRSpeedBuffer::ICRSpeedBuffer()
{

m_curIndex=0;
}

ICRSpeedBuffer::~ICRSpeedBuffer()
{
    // TODO Auto-generated destructor stub
}

void ICRSpeedBuffer::addICRSpeed(ICRSpeed speed, double dt)
{
  m_ICRSpeedList[m_curIndex]=speed;
  m_dtList[m_curIndex]=dt;
  m_curIndex=(m_curIndex+1)%BUFFERSIZE;
}

void ICRSpeedBuffer::removeICRSpeed(ICRSpeed & speed, double & dt)
{
    m_curIndex=(m_curIndex-1 + BUFFERSIZE)%BUFFERSIZE; // be careful to modulo on -1
    speed=m_ICRSpeedList[m_curIndex];
    dt=m_dtList[m_curIndex];
}

