/*
 * uTest_Split.cpp
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

BOOST_AUTO_TEST_CASE( ImproveSegment_Test )
{
    srand(time(0));

    double nbMeas = 50;
    for(unsigned i = 0; i < 100; i++)
    {
        CornerDetector cd;

        double d = (double) rand()/RAND_MAX;
        double alpha = (double) rand()/RAND_MAX * 2. * PI;
        double angleBegin = fmod(alpha - (double) rand()/RAND_MAX * PI / 5.,2 * PI);
        double angleEnd = fmod(alpha + (double) rand()/RAND_MAX * PI / 5.,2 * PI);

        SegmentedScan polarScan(3, 3 * nbMeas);
        for (unsigned int i = 0; i < nbMeas; i++)
        {
            double angle = fmod( angleBegin + i * (angleEnd - angleBegin) / (nbMeas - 1) , 2*PI );
            polarScan(0, i) = angle;
            polarScan(1, i) = d / cos(alpha - angle);
            polarScan(2, i) = 0;
        }

        for (unsigned int i = nbMeas; i < 2 * nbMeas; i++)
        {
            double angle = fmod( angleEnd + i * (angleEnd - angleBegin) / (nbMeas - 1) , 2*PI );
            polarScan(0, i) = angle;
            polarScan(1, i) = (double) rand()/RAND_MAX;
            polarScan(2, i) = -1;
        }

        for (unsigned int i = 2 *nbMeas; i < 3 * nbMeas; i++)
        {
            double angle = fmod( 2 * angleEnd - angleBegin + i * (angleEnd - angleBegin) / (nbMeas - 1) , 2*PI );
            polarScan(0, i) = angle;
            polarScan(1, i) = d / cos(alpha - angle);
            polarScan(2, i) = 1;
        }

        std::pair<Segment, SegmentedScan> p = cd.improveSegment(polarScan, std::make_pair(0,1));

        BOOST_CHECK_CLOSE( p.first.d, d , 0.001f);
        BOOST_CHECK_CLOSE( p.first.alpha, alpha, 0.001f );
        BOOST_CHECK_EQUAL( p.first.nbMeas, 2 * nbMeas );
    }
}
