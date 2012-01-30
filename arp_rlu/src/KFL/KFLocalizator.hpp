/*
 * KFLocalizator.hpp
 *
 *  Created on: 22 January 2012
 *      Author: Boris
 */

#ifndef _ARP_RLU_KFL_KFLOCALIZATOR_HPP_
#define _ARP_RLU_KFL_KFLOCALIZATOR_HPP_

#include <math/core>

#include <KFL/BeaconDetector.hpp>
#include <KFL/BayesianWrapper.hpp>

namespace arp_rlu
{

namespace kfl
{

class KFLocalizator
{
    public:
        class InitParams
        {
            public:
                InitParams();
                std::string getInfo();

                arp_math::EstimatedPose2D initialPose;
        };

        class IEKFParams
        {
            public:
                IEKFParams();
                std::string getInfo();

                double defaultOdoVelTransSigma;
                double defaultOdoVelRotSigma;
                double defaultLaserRangeSigma;
                double defaultLaserThetaSigma;
                unsigned int iekfMaxIt;
        };

        class Params
        {
            public:
                Params();
                std::string getInfo();

                unsigned int bufferSize;
                kfl::KFLocalizator::InitParams initParams;
                kfl::KFLocalizator::IEKFParams iekfParams;
                kfl::BeaconDetector::Params procParams;
        };

    public:
        KFLocalizator();
        ~KFLocalizator();

        void setParams(KFLocalizator::Params);
        bool setParams(KFLocalizator::InitParams);
        void setParams(KFLocalizator::IEKFParams);
        void setParams(BeaconDetector::Params);
        bool initialize();
        bool newOdoVelocity(double time, arp_math::Twist2D odoVel);
        bool newScan(double time, lsl::LaserScan scan);
        arp_math::EstimatedPose2D getPose2D();
        arp_math::EstimatedTwist2D getTwist2D();


    protected:
        KFLocalizator::Params params;
        BeaconDetector beaconDetector;
        BayesianWrapper * baysesian;



};

} // namespace kfl
} // namespace arp_rlu

#endif /* _ARP_RLU_KFL_KFLOCALIZATOR_HPP_ */
