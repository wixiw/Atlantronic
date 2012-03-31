/*
 * UbiquityKinematics.hpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#ifndef UBIQUITYKINEMATICS_HPP_
#define UBIQUITYKINEMATICS_HPP_

namespace arp_core
{

class UbiquityKinematics
{
    public:
        UbiquityKinematics();
        virtual ~UbiquityKinematics();

        static void turrets2Motors();
        static void motors2Turrets();
        static void twist2Turrets();
        static void turrets2Twist();
};

} /* namespace arp_core */
#endif /* UBIQUITYKINEMATICS_HPP_ */
