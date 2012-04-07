/*
 * ParamsComponent.hpp
 *
 *  Created on: Apr 1, 2012
 *      Author: ard
 */

#ifndef ParamsComponent_HPP_
#define ParamsComponent_HPP_

#include "taskcontexts/ARDTaskContext.hpp"
#include "models/UbiquityParams.hpp"

namespace arp_core
{

class ParamsComponent: public ARDTaskContext
{
    public:
        ParamsComponent(const std::string& name);
        bool configureHook();
        void updateHook();

    protected:
        arp_model::UbiquityParams propSavedParams;
        RTT::OutputPort<arp_model::UbiquityParams> outParams;
};

} /* namespace arp_core */
#endif /* UbiquityParams_HPP_ */
