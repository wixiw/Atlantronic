/*
 * UbiquityParams.hpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#ifndef UbiquityParams_HPP_
#define UbiquityParams_HPP_

#include "taskcontexts/ARDTaskContext.hpp"

namespace arp_core
{

class UbiquityParams: public ARDTaskContext
{
    public:
        UbiquityParams(const std::string& name);

    protected:
};

} /* namespace arp_core */
#endif /* UbiquityParams_HPP_ */
