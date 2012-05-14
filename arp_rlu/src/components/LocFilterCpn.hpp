/*
 * LocFilterCpn.hpp
 *
 *  Created on: 9 mai 2012
 *      Author: ard
 */

#ifndef LocFilterCpn_HPP_
#define LocFilterCpn_HPP_

#include "RluTaskContext.hpp"
#include <math/core>


namespace arp_rlu
{

class LocFilterCpn: public RluTaskContext
{
    public:
    LocFilterCpn(const std::string& name);
    std::string printParams();
    bool configureHook();
    void updateHook();

    protected:
    //*****************************************************
    // Params
    double propCutOffFrequency;
    double propSamplingFrequency;

    //*****************************************************
    // Ports
    RTT::InputPort<arp_math::EstimatedPose2D> inPose;

    RTT::OutputPort<arp_math::EstimatedPose2D> outPose;

    //*****************************************************
    // Methods

    /* Cree l'interface Orocos : ajout de port, proprietes, operations */
    void createOrocosInterface();



    //*****************************************************
    // Operations
    virtual std::string coGetPerformanceReport();

    bool ooReset();

    arp_math::EstimatedPose2D X_nm1;
    arp_math::EstimatedPose2D X_nm2;
    arp_math::EstimatedPose2D Y_nm1;
    arp_math::EstimatedPose2D Y_nm2;

    bool initialized;

};

} /* namespace arp_rlu */
#endif /* LocFilterCpn_HPP_ */
