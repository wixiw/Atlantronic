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
    ss << " [*] radiusTolerance: " << radiusTolerance << " (m)" << std::endl;
    ss << " [*] distanceTolerance: " << distanceTolerance << " (m)" << std::endl;
    ss << " [*] lengthTolerance: " << lengthTolerance << " (m)" << std::endl;
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

    std::vector< Eigen::VectorXi > combs = combinaisons(vdc.size(), 2);
    for(unsigned int i = 0 ; i < combs.size() ; i++)
    {
    }

    for(std::vector<std::pair<Circle, Circle> >::const_iterator prcIt = vprcrc.begin() ; prcIt != vprcrc.end() ; ++prcIt)
    {
        Eigen::Vector2d refVect = prcIt->first.getPosition() - prcIt->second.getPosition();
        double refLength = refVect.norm();

        std::pair< DetectedCircle, DetectedCircle > candidates;
        std::pair< Circle, Circle > references;
        double bestDeltaLength = -2.;
        double secondBestDeltaLength = -2.;


        std::vector< Eigen::VectorXi > combs = combinaisons(vdc.size(), 2);
        for(unsigned int k = 0 ; k < combs.size() ; k++)
        {
            std::vector<DetectedCircle>::const_iterator dcIt1( &vdc[combs[k](0)] );
            std::vector<DetectedCircle>::const_iterator dcIt2( &vdc[combs[k](1)] );
            if( dcIt2 == dcIt1 )
            {
                continue;
            }

            Eigen::Vector2d vect = dcIt2->getPosition() - dcIt1->getPosition();
            double length = vect.norm();
            if( abs(length - refLength) > p.lengthTolerance )
            {
                continue;
            }

            if( bestDeltaLength > -1. )
            {
                if( abs(length -refLength) > bestDeltaLength )
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

            std::vector< std::pair<DetectedCircle, Circle> > sorted;
            if( p.distanceTolerance > 0)
            {
                SoloCircleIdentif::Params soloParams;
                soloParams.radiusTolerance = p.radiusTolerance;
                soloParams.distanceTolerance = p.distanceTolerance;
                sorted = SoloCircleIdentif::apply(dc, rc, soloParams);
            }
            else
            {
                Vector3 aDC(dc[0].x(), dc[0].y(), 0.);
                Vector3 bDC(dc[1].x(), dc[1].y(), 0.);
                Vector3 aRC(rc[0].x(), rc[0].y(), 0.);
                Vector3 bRC(rc[1].x(), rc[1].y(), 0.);
                if( aRC.cross(bRC)(2) >= 0 ) // les points de référence sont dans le sens trigo
                        {
                    if( aDC.cross(bDC)(2) > 0 ) // les points mesurés sont dans le sens trigo
                    {
                        sorted.push_back( std::make_pair( dc[0], rc[0] ) );
                        sorted.push_back( std::make_pair( dc[1], rc[1] ) );
                    }
                    else  // les points mesurés ne sont pas dans le sens trigo
                    {
                        sorted.push_back( std::make_pair( dc[1], rc[0] ) );
                        sorted.push_back( std::make_pair( dc[0], rc[1] ) );
                    }
                        }
                else  // les points de référence ne sont pas dans le sens trigo
                {
                    if( aDC.cross(bDC)(2) > 0 ) // les points mesurés sont dans le sens trigo
                    {
                        sorted.push_back( std::make_pair( dc[0], rc[1] ) );
                        sorted.push_back( std::make_pair( dc[1], rc[0] ) );
                    }
                    else  // les points mesurés ne sont pas dans le sens trigo
                    {
                        sorted.push_back( std::make_pair( dc[1], rc[1] ) );
                        sorted.push_back( std::make_pair( dc[0], rc[0] ) );
                    }
                }
            }
            if( sorted.size() != 2 )
            {
                continue;
            }

            secondBestDeltaLength = bestDeltaLength;
            bestDeltaLength = abs(length -refLength);
            candidates.first = sorted[0].first;
            candidates.second = sorted[1].first;
            references.first = sorted[0].second;
            references.second = sorted[1].second;
        }
        if(bestDeltaLength > -1.)
        {
            if( (secondBestDeltaLength < -1.) || (secondBestDeltaLength > 5. * bestDeltaLength))
            {
                Log( DEBUG ) << "DuoCircleIdentif::apply - Segment " << references.first.toString() << " -- " << references.second.toString() << " succeed :";
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] " << candidates.first.toString();
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] " << candidates.second.toString();
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] bestDeltaLength : " << bestDeltaLength;
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] secondBestDeltaLength : " << secondBestDeltaLength;
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] measure length before correction : " << (candidates.first.getPosition() - candidates.second.getPosition()).norm();
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] reference length : " << (references.first.getPosition() - references.second.getPosition()).norm();

                // Correction Thales
                double ratio = (references.first.getPosition() - references.second.getPosition()).norm() / (candidates.first.getPosition() - candidates.second.getPosition()).norm();
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] ratio : " << ratio;
                candidates.first.setApparentCenterRange( ratio * candidates.first.getApparentCenterRange() );
                candidates.second.setApparentCenterRange( ratio * candidates.second.getApparentCenterRange() );


//                // Correction Al Kashi
//                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] first meas before correction : (" << candidates.first.getApparentCenterRange() << " , " << candidates.first.getApparentCenterTheta() << " )";
//                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] second meas before correction : (" << candidates.second.getApparentCenterRange() << " , " << candidates.second.getApparentCenterTheta() << " )";
//                double ratio = (references.first.getPosition() - candidates.first.getPointOfView()).norm()
//                             / (references.second.getPosition() - candidates.second.getPointOfView()).norm();
//                double refBase = (references.first.getPosition() - references.second.getPosition()).norm();
//                double gamma = betweenMinusPiAndPlusPi( candidates.second.getApparentCenterTheta() - candidates.first.getApparentCenterTheta());
//                candidates.second.setApparentCenterRange( sqrt( refBase*refBase / (ratio*ratio + 1. - 2. * ratio * cos(gamma)) )  );
//                candidates.first.setApparentCenterRange( ratio * candidates.second.getApparentCenterRange() );
//                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] bestDeltaLength : " << bestDeltaLength;
//                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] ratio : " << ratio;
//                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] first meas after correction : (" << candidates.first.getApparentCenterRange() << " , " << candidates.first.getApparentCenterTheta() << " )";
//                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] second meas after correction : (" << candidates.second.getApparentCenterRange() << " , " << candidates.second.getApparentCenterTheta() << " )";


                // Log
                Vector2 newSegmt;
                newSegmt(0) = candidates.first.getApparentCenterRange() * cos(candidates.first.getApparentCenterTheta())
                            - candidates.second.getApparentCenterRange() * cos(candidates.second.getApparentCenterTheta());
                newSegmt(1) = candidates.first.getApparentCenterRange() * sin(candidates.first.getApparentCenterTheta())
                            - candidates.second.getApparentCenterRange() * sin(candidates.second.getApparentCenterTheta());
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] measure length after correction : " << newSegmt.norm();

                // Constitution du résultat
                out.push_back( std::make_pair( std::make_pair(candidates.first, candidates.second), std::make_pair(references.first, references.second) ));
            }
            else
            {
                Log( DEBUG ) << "DuoCircleIdentif::apply - Segment " << references.first.toString() << " -- " << references.second.toString() << " failed :";
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] " << candidates.first.toString();
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] " << candidates.second.toString();
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] bestDeltaLength : " << bestDeltaLength;
                Log( DEBUG ) << "DuoCircleIdentif::apply -    [*] secondBestDeltaLength : " << secondBestDeltaLength;
            }
        }
    }

    return out;
}

