/*
 * uTest_ComputeSegment.cpp
 *
 *  Created on: 12 sept. 2010
 *      Author: boris
 */


#include "cornerdetector/CornerDetector.hpp"
#include <math/math.hpp>

using namespace arp_math;
using namespace Eigen;
using namespace arp_rlu;
using namespace std;

BOOST_AUTO_TEST_CASE( Compute_Segment_nbMeas_100_Test )
{
    srand(time(0));

    double nbMeas = 100;
    for(unsigned i = 0 ; i < 100 ; i++)
    {
        CornerDetector cd;

        double d = (double) rand()/RAND_MAX;
        double alpha = (double) rand()/RAND_MAX * 2. * PI;
        double angleBegin = fmod(alpha - (double) rand()/RAND_MAX * PI / 2.,2 * PI);
        double angleEnd = fmod(alpha + (double) rand()/RAND_MAX * PI / 2.,2 * PI);

        MatrixXd polarScan(2, nbMeas);
        for (unsigned int i = 0; i < nbMeas; i++)
        {
            double angle = fmod( angleBegin + i * (angleEnd - angleBegin) / (nbMeas - 1) , 2*PI );
            polarScan(0, i) = angle;
            polarScan(1, i) = d / cos(alpha - angle);
        }

        Segment s = cd.computeSegment(polarScan);

        BOOST_CHECK_CLOSE( s.d, d , 0.001f);
        BOOST_CHECK_CLOSE( s.alpha, alpha, 0.001f );
        BOOST_CHECK_EQUAL( s.angleBegin, angleBegin );
        BOOST_CHECK_EQUAL( s.angleEnd, angleEnd );
        BOOST_CHECK_EQUAL( s.nbMeas, nbMeas );
    }
}


BOOST_AUTO_TEST_CASE( Compute_Segment_nbMeas_2_Test )
{
    srand(time(0));

    double nbMeas = 2;
    for(unsigned i = 0; i < 100; i++)
    {
        CornerDetector cd;

        double d = (double) rand()/RAND_MAX;
        double alpha = (double) rand()/RAND_MAX * 2. * PI;
        double angleBegin = fmod(alpha - (double) rand()/RAND_MAX * PI / 2.,2 * PI);
        double angleEnd = fmod(alpha + (double) rand()/RAND_MAX * PI / 2.,2 * PI);

        MatrixXd polarScan(2, nbMeas);
        for (unsigned int i = 0; i < nbMeas; i++)
        {
            double angle = fmod( angleBegin + i * (angleEnd - angleBegin) / (nbMeas - 1) , 2*PI );
            polarScan(0, i) = angle;
            polarScan(1, i) = d / cos(alpha - angle);
        }

        Segment s = cd.computeSegment(polarScan);

        BOOST_CHECK_CLOSE( s.d, d , 0.001f);
        BOOST_CHECK_CLOSE( s.alpha, alpha, 0.001f );
        BOOST_CHECK_EQUAL( s.angleBegin, angleBegin );
        BOOST_CHECK_EQUAL( s.angleEnd, angleEnd );
        BOOST_CHECK_EQUAL( s.nbMeas, nbMeas );
    }
}

