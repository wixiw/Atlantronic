/*
 * TwistBuffer.cpp
 *
 *  Created on: Apr 30, 2012
 *      Author: Romain Moulin
 */

#include "TwistBuffer.hpp"
#include <iostream>

using namespace arp_core::log;
using namespace arp_model;
using namespace arp_math;
using namespace arp_ods;
using namespace std;

TwistBuffer::TwistBuffer()
{

m_curIndex=0;
}

TwistBuffer::~TwistBuffer()
{
    // TODO Auto-generated destructor stub
}

void TwistBuffer::addTwist(Twist2D twist, double dt)
{
  m_twistList[m_curIndex]=twist;
  m_dtList[m_curIndex]=dt;
  m_curIndex=(m_curIndex+1)%BUFFERSIZE;
}

void TwistBuffer::removeTwist(Twist2D & twist, double & dt)
{
    m_curIndex=(m_curIndex-1 + BUFFERSIZE)%BUFFERSIZE; // be careful to modulo on -1
    twist=m_twistList[m_curIndex];
    dt=m_dtList[m_curIndex];
}

