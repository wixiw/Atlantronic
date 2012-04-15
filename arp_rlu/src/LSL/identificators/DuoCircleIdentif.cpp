/*
 * DuoCircleIdentif.cpp
 *
 *  Created on: 23 Mars 2012
 *      Author: Boris
 */

#include "DuoCircleIdentif.hpp"

#include "LSL/Logger.hpp"

#include <LSL/identificators/SoloCircleIdentif.hpp>

#include <exceptions/NotImplementedException.hpp>


using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace lsl;
using namespace arp_core::log;

DuoCircleIdentif::Params::Params()
: radiusTolerance(0.05)
, distanceTolerance(0.3)
, lengthTolerance(0.1)
{
}

std::string DuoCircleIdentif::Params::getInfo() const
{
    std::stringstream ss;
    ss << "DuoCircleIdentif params :" << std::endl;
    ss << " [*] radiusTolerance: " << radiusTolerance << std::endl;
    ss << " [*] distanceTolerance: " << distanceTolerance << std::endl;
    ss << " [*] lengthTolerance: " << lengthTolerance << std::endl;
    return ss.str();
}

bool DuoCircleIdentif::Params::checkConsistency() const
{
    if( radiusTolerance < 0 )
    {
        Log( WARN ) << "DuoCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (radiusTolerance < 0)";
        return false;
    }
    if( distanceTolerance < 0 )
    {
        Log( WARN ) << "DuoCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (distanceTolerance < 0)";
        return false;
    }
    if( lengthTolerance < 0 )
    {
        Log( WARN ) << "DuoCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (lengthTolerance < 0)";
        return false;
    }
    return true;
}

std::vector< std::pair< std::pair<DetectedCircle, DetectedCircle>, std::pair<Circle,Circle> > > DuoCircleIdentif::apply(const std::vector<DetectedCircle> & vdc, const std::vector< std::pair<Circle,Circle> > & vprcrc, const Params & p)
{
    std::vector< std::pair< std::pair<DetectedCircle, DetectedCircle>, std::pair<Circle,Circle> > > out;

    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "DuoCircleIdentif::apply" << " - " << "Parameters are not consistent => Return empty";
        return out;
    }

    for(std::vector<std::pair<Circle, Circle> >::const_iterator prcIt = vprcrc.begin() ; prcIt != vprcrc.end() ; ++prcIt)
    {
        Eigen::Vector2d refVect = prcIt->first.getPosition() - prcIt->second.getPosition();
        double refLength = sqrt( refVect[0]*refVect[0] + refVect[1]*refVect[1] );

        std::pair< DetectedCircle, DetectedCircle > candidates;
        double bestLength = -2.;

        for(std::vector<DetectedCircle>::const_iterator dcIt1 = vdc.begin() ; dcIt1 != vdc.end() ; ++dcIt1)
        {
            for(std::vector<DetectedCircle>::const_iterator dcIt2 = dcIt1 ; dcIt2 != vdc.end() ; ++dcIt2)
            {
                if( dcIt2 == dcIt1 )
                {
                    continue;
                }

                Eigen::Vector2d vect = dcIt2->getPosition() - dcIt1->getPosition();
                double length = sqrt( vect[0]*vect[0] + vect[1]*vect[1] );
                if( abs(length - refLength) > p.lengthTolerance )
                {
//                    Log( DEBUG ) << "DuoCircleIdentif::apply - Segment rejected because deltaLength (" << abs(length - refLength) << ") > p.lengthTolerance (" << p.lengthTolerance << ")";
                    continue;
                }

                if( bestLength > -1. )
                {
                    if( abs(length -refLength) > abs(bestLength -refLength) )
                    {
                        continue;
                    }
                }

                std::vector<DetectedCircle> dc;
                dc.push_back(*dcIt1);
                dc.push_back(*dcIt2);
                std::vector<Circle> rc;
                rc.push_back(prcIt->first);
                rc.push_back(prcIt->second);
                SoloCircleIdentif::Params soloParams;
                soloParams.radiusTolerance = p.radiusTolerance;
                soloParams.distanceTolerance = p.distanceTolerance;
                std::vector< std::pair<DetectedCircle, Circle> > sorted = SoloCircleIdentif::apply(dc, rc, soloParams);
                if( sorted.size() != 2 )
                {
//                    Log( DEBUG ) << "DuoCircleIdentif::apply - Segment rejected because it did not passed SoloCircleIdentif";
                    continue;
                }

                bestLength = length;
                candidates.first = sorted[0].first;
                candidates.second = sorted[1].first;
            }
        }
        if(bestLength > -1.)
        {
            out.push_back( std::make_pair( std::make_pair(candidates.first, candidates.second), std::make_pair(prcIt->first, prcIt->second) ));
        }
    }

    return out;
}

