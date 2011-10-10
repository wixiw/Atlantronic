/*
 * UbiquityItf.hpp
 *
 *  Created on: 03 Oct 2011
 *      Author: wla
 *
 *  This is the simulation of Ubiquity's hardware interface
 */

#ifndef UBIQUITYSIMUL_HPP_
#define UBIQUITYSIMUL_HPP_

#include "orocos/platform/UbiquityItf.hpp"

namespace arp_hml
{

    class UbiquitySimul: public UbiquityItf
    {
    public:
        UbiquitySimul(const std::string& name);

    protected:
        /**
         * Redefined to prevent getOperation from inheritance
         */
        bool configureHook();

        /**
         *
         */
        void updateHook();

        void loopEncoder();
    };

}

#endif
