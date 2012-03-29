/*
 * SoloCircleIdentif.cpp
 *
 *  Created on: 23 Mars 2012
 *      Author: Boris
 */

#include "SoloCircleIdentif.hpp"

#include "LSL/Logger.hpp"

#include <exceptions/NotImplementedException.hpp>

#include <numeric>
#include <algorithm>


using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace lsl;
using namespace arp_core::log;

SoloCircleIdentif::Params::Params()
: radiusTolerance(0.05)
, distanceTolerance(0.1)
{
}

std::string SoloCircleIdentif::Params::getInfo() const
{
    std::stringstream ss;
    ss << "SoloCircleIdentif params :" << std::endl;
    ss << " [*] radiusTolerance: " << radiusTolerance << std::endl;
    ss << " [*] distanceTolerance: " << distanceTolerance << std::endl;
    return ss.str();
}

bool SoloCircleIdentif::Params::checkConsistency() const
{
    if( radiusTolerance < 0 )
    {
        Log( WARN ) << "SoloCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (radiusTolerance < 0)";
        return false;
    }
    if( distanceTolerance < 0 )
    {
        Log( WARN ) << "SoloCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (distanceTolerance < 0)";
        return false;
    }
    return true;
}

std::vector< std::pair<DetectedCircle, Circle> > SoloCircleIdentif::apply(const std::vector<DetectedCircle> & vdc, const std::vector<Circle> & vrc, const Params & p)
{
    std::vector< std::pair<DetectedCircle, Circle> > out;

    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "SoloCircleIdentif::apply" << " - " << "Parameters are not consistent => Return empty";
        return out;
    }

    for( std::vector<Circle>::const_iterator rcIt = vrc.begin() ; rcIt != vrc.end() ; ++rcIt)
    {
        double minDist = 0.;
        std::vector<DetectedCircle>::const_iterator bestDcIt = vdc.end();
        for( std::vector<DetectedCircle>::const_iterator dcIt = vdc.begin() ; dcIt != vdc.end() ; ++dcIt )
        {
            if( abs(rcIt->r() - dcIt->r()) > p.radiusTolerance )
            {
//                Log( DEBUG ) << "SoloCircleIdentif::apply - deltaRadius > p.distanceTolerance (" << abs(rcIt->r() - dcIt->r()) << " > " << p.radiusTolerance << ") => DetectedCircle( x=" << dcIt->x() <<", y=" << dcIt->y() << ", r=" << dcIt->r() << " ) is rejected";
                continue;
            }
            Eigen::Vector2d vect = rcIt->getPosition() - dcIt->getPosition();
            double dist = sqrt( vect[0]*vect[0] + vect[1]*vect[1] );
            if( dist > p.distanceTolerance )
            {
//                Log( DEBUG ) << "SoloCircleIdentif::apply - distance > p.distanceTolerance (" << dist << " > " << p.distanceTolerance << ") => DetectedCircle( x=" << dcIt->x() <<", y=" << dcIt->y() << ", r=" << dcIt->r() << " ) is rejected";
                continue;
            }
            if(bestDcIt != vdc.end())
            {
                if(dist > minDist)
                {
                    continue;
                }
            }
            bestDcIt = dcIt;
            minDist = dist;
        }
        if(bestDcIt == vdc.end())
        {
            continue;
        }
        out.push_back( std::make_pair( *bestDcIt, *rcIt ) );
    }

    return out;
}

