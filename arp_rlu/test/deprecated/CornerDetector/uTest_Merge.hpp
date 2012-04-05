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

BOOST_AUTO_TEST_CASE( Merge_Test )
{
    srand(time(0));
    for(unsigned i = 0; i < 100; i++)
    {
        CornerDetector cd;

        unsigned int nbMeas = 50;

        double d = (double) rand() / RAND_MAX;
        double alpha = (double) rand() / RAND_MAX * 2. * PI;
        double angleBegin = fmod(alpha - (double) rand() / RAND_MAX * PI / 5., 2 * PI);
        double angleEnd = fmod(alpha + (double) rand() / RAND_MAX * PI / 5., 2 * PI);

        Segment s;
        s.d = d;
        s.alpha = alpha;
        s.angleBegin = angleBegin;
        s.angleEnd = angleEnd;
        s.nbMeas = nbMeas;

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

        std::vector<Segment> sgmts;
        sgmts.push_back(s);
        sgmts.push_back(s);

        std::pair<std::vector<Segment>, SegmentedScan> p;
        p = cd.merge(sgmts, polarScan, 0.01, 0.01);

        BOOST_CHECK( p.first.size() == 3);
        if( p.first.size() == 3 )
        {
            BOOST_CHECK_CLOSE( p.first[2].d, d, 1.f );
            BOOST_CHECK_CLOSE( p.first[2].alpha, alpha, 1.f );
        }
    }
}
