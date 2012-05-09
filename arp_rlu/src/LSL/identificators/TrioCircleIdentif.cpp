/*
 * TrioCircleIdentif.cpp
 *
 *  Created on: 23 Mars 2012
 *      Author: Boris
 */

#include "TrioCircleIdentif.hpp"

#include "LSL/Logger.hpp"

#include <LSL/identificators/SoloCircleIdentif.hpp>

#include <exceptions/NotImplementedException.hpp>


using namespace arp_math;
using namespace arp_rlu;
using namespace std;
using namespace lsl;
using namespace arp_core::log;

TrioCircleIdentif::Params::Params()
: radiusTolerance(0.05)
, distanceTolerance(0.3)
, maxLengthTolerance(0.1)
, medLengthTolerance(0.1)
, minLengthTolerance(0.1)
{
}

std::string TrioCircleIdentif::Params::getInfo() const
{
    std::stringstream ss;
    ss << "TrioCircleIdentif params :" << std::endl;
    ss << " [*] radiusTolerance: " << radiusTolerance << " (m)" << std::endl;
    ss << " [*] distanceTolerance: " << distanceTolerance << " (m)" << std::endl;
    ss << " [*] maxLengthTolerance: " << maxLengthTolerance << " (m)" << std::endl;
    ss << " [*] medLengthTolerance: " << medLengthTolerance << " (m)" << std::endl;
    ss << " [*] minLengthTolerance: " << minLengthTolerance << " (m)" << std::endl;
    return ss.str();
}

bool TrioCircleIdentif::Params::checkConsistency() const
{
    if( radiusTolerance < 0 )
    {
        Log( WARN ) << "TrioCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (radiusTolerance < 0)";
        return false;
    }
    if( distanceTolerance < 0 )
    {
        Log( WARN ) << "TrioCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (distanceTolerance < 0)";
        return false;
    }
    if( maxLengthTolerance < 0 )
    {
        Log( WARN ) << "TrioCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (maxLengthTolerance < 0)";
        return false;
    }
    if( medLengthTolerance < 0 )
    {
        Log( WARN ) << "TrioCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (medLengthTolerance < 0)";
        return false;
    }
    if( minLengthTolerance < 0 )
    {
        Log( WARN ) << "TrioCircleIdentif::Params::checkConsistency" << " - " << "inconsistent parameters (minLengthTolerance < 0)";
        return false;
    }
    return true;
}

std::vector< std::pair< std::vector<DetectedCircle>, std::vector<Circle> > > TrioCircleIdentif::apply(const std::vector<DetectedCircle> & vdc, const std::vector< std::vector<Circle> > & vrc, const Params & p)
{
    std::vector< std::pair< std::vector<DetectedCircle>, std::vector<Circle> > > out;

    if( !p.checkConsistency() )
    {
        Log( ERROR ) << "TrioCircleIdentif::apply" << " - " << "Parameters are not consistent => Return empty";
        return std::vector< std::pair< std::vector<DetectedCircle>, std::vector<Circle> > >();
    }

    if( vdc.size() < 3 )
    {
        Log( DEBUG ) << "TrioCircleIdentif::apply - Less than 3 DetectedCircle (actually " << vdc.size() << ")  => return empty";
        return std::vector< std::pair< std::vector<DetectedCircle>, std::vector<Circle> > >();
    }

    for( std::vector<std::vector<Circle> >::const_iterator vrcIt = vrc.begin() ; vrcIt != vrc.end() ; ++vrcIt )
    {
        if( vrcIt->size() != 3 )
        {
//            Log( WARN ) << "TrioCircleIdentif::apply - Triangle rejected because reference Circle vector is not size 3 (actually " << vrcIt->size() << ")";
            continue;
        }

        Circle rc1 = (*vrcIt)[0];
        Circle rc2 = (*vrcIt)[1];
        Circle rc3 = (*vrcIt)[2];

        std::vector<Circle> rc;
        rc.push_back(rc1);
        rc.push_back(rc2);
        rc.push_back(rc3);

        Eigen::Vector2d refEdge23 = rc3.getPosition() - rc2.getPosition();
        Eigen::Vector2d refEdge13 = rc3.getPosition() - rc1.getPosition();
        Eigen::Vector2d refEdge12 = rc2.getPosition() - rc1.getPosition();
        double refLength23 = sqrt( refEdge23[0]*refEdge23[0] + refEdge23[1]*refEdge23[1] );
        double refLength13 = sqrt( refEdge13[0]*refEdge13[0] + refEdge13[1]*refEdge13[1] );
        double refLength12 = sqrt( refEdge12[0]*refEdge12[0] + refEdge12[1]*refEdge12[1] );
        Eigen::Vector3d refLengths(refLength23, refLength13, refLength12);
        int iRefMin, iRefMax;
        refLengths.minCoeff(&iRefMin);
        refLengths.maxCoeff(&iRefMax);
        int iRefMed = 3 - iRefMin - iRefMax;

        std::vector< DetectedCircle > candidates;
        double quality = -2.;

        for(std::vector<DetectedCircle>::const_iterator dcIt1 = vdc.begin() ; dcIt1 != vdc.end() ; ++dcIt1)
        {
            for(std::vector<DetectedCircle>::const_iterator dcIt2 = vdc.begin() ; dcIt2 != vdc.end() ; ++dcIt2)
            {
                if( dcIt2 == dcIt1 )
                {
                    continue;
                }
                for(std::vector<DetectedCircle>::const_iterator dcIt3 = vdc.begin() ; dcIt3 != vdc.end() ; ++dcIt3)
                {
                    if( dcIt3 == dcIt2 || dcIt3 == dcIt1)
                    {
                        continue;
                    }
                    Eigen::Vector2d edge23 = dcIt2->getPosition() - dcIt3->getPosition();
                    Eigen::Vector2d edge13 = dcIt1->getPosition() - dcIt3->getPosition();
                    Eigen::Vector2d edge12 = dcIt1->getPosition() - dcIt2->getPosition();
                    double length23 = sqrt( edge23[0]*edge23[0] + edge23[1]*edge23[1] );
                    double length13 = sqrt( edge13[0]*edge13[0] + edge13[1]*edge13[1] );
                    double length12 = sqrt( edge12[0]*edge12[0] + edge12[1]*edge12[1] );
                    Eigen::Vector3d lengths(length23, length13, length12);
                    int iMin, iMax;
                    lengths.minCoeff(&iMin);
                    lengths.maxCoeff(&iMax);
                    int iMed = 3 - iMin - iMax;
                    if( iMed >= 3 || iMed < 0)
                    {
//                        Log( DEBUG ) << "TrioCircleIdentif::apply - Triangle rejected because triangle is equilateral" ;
                        continue;
                    }

                    if( abs(lengths[iMax] - refLengths[iRefMax]) > p.maxLengthTolerance )
                    {
//                        Log( DEBUG ) << "TrioCircleIdentif::apply - Triangle rejected because deltaMaxLength (" << abs(lengths[iMax] - refLengths[iRefMax]) << " > " << p.maxLengthTolerance << ")" ;
                        continue;
                    }

                    if( abs(lengths[iMed] - refLengths[iRefMed]) > p.medLengthTolerance )
                    {
//                        Log( DEBUG ) << "TrioCircleIdentif::apply - Triangle rejected because deltaMedLength (" << abs(lengths[iMed] - refLengths[iRefMed]) << " > " << p.medLengthTolerance << ")" ;
                        continue;
                    }

                    if( abs(lengths[iMin] - refLengths[iRefMin]) > p.minLengthTolerance )
                    {
//                        Log( DEBUG ) << "TrioCircleIdentif::apply - Triangle rejected because deltaMinLength (" << abs(lengths[iMin] - refLengths[iRefMin]) << " > " << p.minLengthTolerance << ")" ;
                        continue;
                    }

                    Eigen::Vector3d delta;
                    delta[0] = p.maxLengthTolerance - abs(lengths[iMax] - refLengths[iRefMax]);
                    delta[1] = p.medLengthTolerance - abs(lengths[iMed] - refLengths[iRefMed]);
                    delta[2] = p.minLengthTolerance - abs(lengths[iMin] - refLengths[iRefMin]);
                    double q = delta.minCoeff();

                    if( quality > -1 )
                    {
                        if( q < quality )
                        {
                            continue;
                        }
                    }

                    std::vector<DetectedCircle> dc;
                    dc.push_back(*dcIt1);
                    dc.push_back(*dcIt2);
                    dc.push_back(*dcIt3);
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
                        sorted = associate(dc, rc);
                    }

                    if( sorted.size() != 3 )
                    {
                        Log( DEBUG ) << "TrioCircleIdentif::apply - Triangle rejected because association failed";
                        continue;
                    }

                    quality = q;
                    candidates.clear();
                    candidates.push_back(sorted[0].first);
                    candidates.push_back(sorted[1].first);
                    candidates.push_back(sorted[2].first);
                }
            }
        }
        if(quality > -1.)
        {
            out.push_back( std::make_pair( candidates, rc ) );
        }
    }


    return out;
}

std::vector< std::pair<DetectedCircle, Circle> > TrioCircleIdentif::associate(const std::vector<DetectedCircle> & vdc, const std::vector<Circle> & vrc)
{
    if( vdc.size() != 3 || vrc.size() != 3 )
        return std::vector< std::pair<DetectedCircle, Circle> >();


    Eigen::Vector3d rA( vrc[0].x(), vrc[0].y(), 0. );
    Eigen::Vector3d rB( vrc[1].x(), vrc[1].y(), 0. );
    Eigen::Vector3d rC( vrc[2].x(), vrc[2].y(), 0. );

    Eigen::Vector3d m1( vdc[0].x(), vdc[0].y(), 0. );
    Eigen::Vector3d m2( vdc[1].x(), vdc[1].y(), 0. );
    Eigen::Vector3d m3( vdc[2].x(), vdc[2].y(), 0. );

    Eigen::Vector3d mA;  Eigen::Vector3d mB;  Eigen::Vector3d mC;

    Eigen::VectorXd M = Eigen::VectorXd::Zero(6);

    // ABC
    mA = m1;    mB = m2;    mC = m3;
    M(0) += ( (rB-rA).cross(rC-rA)/((rB-rA).norm()) - (mB-mA).cross(mC-mA)/((mB-mA).norm()) ).norm();
    M(0) += ( (rC-rB).cross(rA-rB)/((rC-rB).norm()) - (mC-mB).cross(mA-mB)/((mC-mB).norm()) ).norm();
    M(0) += ( (rA-rC).cross(rB-rC)/((rA-rC).norm()) - (mA-mC).cross(mB-mC)/((mA-mC).norm()) ).norm();

    // ACB
    mA = m1;    mC = m2;    mB = m3;
    M(1) += ( (rB-rA).cross(rC-rA)/((rB-rA).norm()) - (mB-mA).cross(mC-mA)/((mB-mA).norm()) ).norm();
    M(1) += ( (rC-rB).cross(rA-rB)/((rC-rB).norm()) - (mC-mB).cross(mA-mB)/((mC-mB).norm()) ).norm();
    M(1) += ( (rA-rC).cross(rB-rC)/((rA-rC).norm()) - (mA-mC).cross(mB-mC)/((mA-mC).norm()) ).norm();

    // BCA
    mB = m1;    mC = m2;    mA = m3;
    M(2) += ( (rB-rA).cross(rC-rA)/((rB-rA).norm()) - (mB-mA).cross(mC-mA)/((mB-mA).norm()) ).norm();
    M(2) += ( (rC-rB).cross(rA-rB)/((rC-rB).norm()) - (mC-mB).cross(mA-mB)/((mC-mB).norm()) ).norm();
    M(2) += ( (rA-rC).cross(rB-rC)/((rA-rC).norm()) - (mA-mC).cross(mB-mC)/((mA-mC).norm()) ).norm();

    // BAC
    mB = m1;    mA = m2;    mC = m3;
    M(3) += ( (rB-rA).cross(rC-rA)/((rB-rA).norm()) - (mB-mA).cross(mC-mA)/((mB-mA).norm()) ).norm();
    M(3) += ( (rC-rB).cross(rA-rB)/((rC-rB).norm()) - (mC-mB).cross(mA-mB)/((mC-mB).norm()) ).norm();
    M(3) += ( (rA-rC).cross(rB-rC)/((rA-rC).norm()) - (mA-mC).cross(mB-mC)/((mA-mC).norm()) ).norm();

    // CBA
    mC = m1;    mB = m2;    mA = m3;
    M(4) += ( (rB-rA).cross(rC-rA)/((rB-rA).norm()) - (mB-mA).cross(mC-mA)/((mB-mA).norm()) ).norm();
    M(4) += ( (rC-rB).cross(rA-rB)/((rC-rB).norm()) - (mC-mB).cross(mA-mB)/((mC-mB).norm()) ).norm();
    M(4) += ( (rA-rC).cross(rB-rC)/((rA-rC).norm()) - (mA-mC).cross(mB-mC)/((mA-mC).norm()) ).norm();

    // CAB
    mC = m1;    mA = m2;    mB = m3;
    M(5) += ( (rB-rA).cross(rC-rA)/((rB-rA).norm()) - (mB-mA).cross(mC-mA)/((mB-mA).norm()) ).norm();
    M(5) += ( (rC-rB).cross(rA-rB)/((rC-rB).norm()) - (mC-mB).cross(mA-mB)/((mC-mB).norm()) ).norm();
    M(5) += ( (rA-rC).cross(rB-rC)/((rA-rC).norm()) - (mA-mC).cross(mB-mC)/((mA-mC).norm()) ).norm();

    std::pair<Eigen::VectorXd, Eigen::VectorXi> p = arp_math::bubbleSortIndices( M );

//    Log( ERROR ) << "M : " << M.transpose();
//    Log( ERROR ) << "p.first : " << p.first.transpose();
//    Log( ERROR ) << "p.second : " << p.second.transpose();

    if( p.first(0) == p.first(1) &&
        p.first(1) == p.first(2) &&
        p.first(3) == p.first(4) &&
        p.first(4) == p.first(5) &&
        p.first(4) > 1000. * p.first(0))
    {
//        Log( ERROR ) << "triangle équilateral";
        return std::vector< std::pair<DetectedCircle, Circle> >();
    }

    std::vector< std::pair<DetectedCircle, Circle> > out;
    switch(p.second(0))
    {
        case 0: //ABC
            out.push_back(std::make_pair(vdc[0], vrc[0]));
            out.push_back(std::make_pair(vdc[1], vrc[1]));
            out.push_back(std::make_pair(vdc[2], vrc[2]));
            break;
        case 1: //ACB
            out.push_back(std::make_pair(vdc[0], vrc[0]));
            out.push_back(std::make_pair(vdc[2], vrc[1]));
            out.push_back(std::make_pair(vdc[1], vrc[2]));
            break;
        case 2://BCA
            out.push_back(std::make_pair(vdc[2], vrc[0]));
            out.push_back(std::make_pair(vdc[0], vrc[1]));
            out.push_back(std::make_pair(vdc[1], vrc[2]));
            break;
        case 3://BAC
            out.push_back(std::make_pair(vdc[1], vrc[0]));
            out.push_back(std::make_pair(vdc[0], vrc[1]));
            out.push_back(std::make_pair(vdc[2], vrc[2]));
            break;
        case 4://CBA
            out.push_back(std::make_pair(vdc[2], vrc[0]));
            out.push_back(std::make_pair(vdc[1], vrc[1]));
            out.push_back(std::make_pair(vdc[0], vrc[2]));
            break;
        case 5://CAB
            out.push_back(std::make_pair(vdc[1], vrc[0]));
            out.push_back(std::make_pair(vdc[2], vrc[1]));
            out.push_back(std::make_pair(vdc[0], vrc[2]));
            break;
        default:
            break;
    }

    return out;
}
