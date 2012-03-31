/*
 * UbiquityParams.hpp
 *
 *  Created on: Mar 31, 2012
 *      Author: ard
 */

#ifndef UBIQUITYPARAMS_HPP_
#define UBIQUITYPARAMS_HPP_

namespace arp_core
{

class UbiquityParams
{
    public:
        UbiquityParams();

        double getLeftTurretZero() const;
        double getRearTurretZero() const;
        double getRightTurretZero() const;
        void setLeftTurretZero(double leftTurretZero);
        void setRearTurretZero(double rearTurretZero);
        void setRightTurretZero(double rightTurretZero);

    protected:
        double m_leftTurretZero;
        double m_rightTurretZero;
        double m_rearTurretZero;
};

} /* namespace arp_core */
#endif /* UBIQUITYPARAMS_HPP_ */
