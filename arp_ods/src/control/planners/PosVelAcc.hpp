/*
 * PosVelAcc.hpp
 *
 *  Created on: Mar 16, 2014
 *      Author: ard
 */

#ifndef POSVELACC_HPP_
#define POSVELACC_HPP_

namespace arp_ods
{

struct PosVelAcc
{
double position;        //m
double velocity;        //m/s
double acceleration;    //m/s2

PosVelAcc(double p = 0., double v = 0., double a = 0.)
{
    position = p;
    velocity = v;
    acceleration = a;
}
};

}



#endif /* POSVELACC_HPP_ */
